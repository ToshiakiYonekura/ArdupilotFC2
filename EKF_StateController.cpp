/*
 * EKF State Feedback Controller Implementation
 * 
 * Implementation of EKF-based state feedback controller for hexacopter
 * with automatic switching between cascade and state feedback control modes.
 */

#include "EKF_StateController.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// ============================================================================
// EKFStateController Implementation
// ============================================================================

EKFStateController::EKFStateController(const HexacopterSystemID& system_id) :
    system_id(system_id),
    ekf3(nullptr),
    ahrs(nullptr),
    current_mode(ControlMode::CASCADE_CONTROL),
    cascade_controller(nullptr),
    current_uncertainty(0.0f),
    adaptive_gain_factor(1.0f),
    mode_switch_count(0),
    last_mode_switch_time_ms(0),
    mode_switch_hysteresis(2000),  // 2 second hysteresis
    reference_natural_freq(1.0f),
    reference_damping_ratio(0.7f),
    wind_compensation_enabled(false)
{
    // Initialize state vectors
    state_estimate.zero();
    state_reference.zero();
    state_error.zero();
    control_output.zero();
    
    // Set default control limits based on system capabilities
    control_limits = Vector4f(
        system_id.max_thrust_per_motor * system_id.arm_length * 3.0f,  // Max roll torque
        system_id.max_thrust_per_motor * system_id.arm_length * 3.0f,  // Max pitch torque
        system_id.max_thrust_per_motor * system_id.torque_to_thrust_ratio * 6.0f,  // Max yaw torque
        system_id.max_thrust_per_motor * 6.0f  // Max total thrust
    );
    
    // Initialize LQR matrices
    Q_matrix.zero();
    R_matrix.zero();
    K_lqr.zero();
    K_adaptive.zero();
    
    // Initialize diagnostics
    diagnostics.active_mode = current_mode;
    diagnostics.estimation_uncertainty = 0.0f;
    diagnostics.ekf_healthy = false;
}

bool EKFStateController::initialize()
{
    // Get EKF3 and AHRS instances
    ahrs = AP::ahrs();
    if (!ahrs) {
        hal.console->printf("EKFStateController: Failed to get AHRS instance\n");
        return false;
    }
    
    ekf3 = ahrs->get_NavEKF3();
    if (!ekf3) {
        hal.console->printf("EKFStateController: EKF3 not available\n");
        return false;
    }
    
    // Initialize LQG matrices
    compute_lqr_cost_matrix();
    compute_control_cost_matrix();
    
    // Compute initial LQR gains (simplified for initialization)
    compute_initial_lqr_gains();
    
    // Initialize cascade controller as fallback
    cascade_controller = new HexacopterControlSystem();
    if (!cascade_controller || !cascade_controller->initialize()) {
        hal.console->printf("EKFStateController: Failed to initialize cascade controller\n");
        return false;
    }
    
    hal.console->printf("EKFStateController: Initialized successfully\n");
    return true;
}

Vector4f EKFStateController::update_control(const Vector3f& position_cmd, 
                                           const Vector3f& velocity_cmd,
                                           const Vector3f& attitude_cmd,
                                           float yaw_rate_cmd,
                                           float dt)
{
    // Update reference state
    set_position_reference(position_cmd, velocity_cmd);
    
    // Extract current EKF state
    if (!extract_ekf_state_vector()) {
        // Fallback to cascade control if EKF state extraction fails
        if (current_mode != ControlMode::CASCADE_CONTROL) {
            hal.console->printf("EKFStateController: EKF state extraction failed, switching to cascade\n");
            set_control_mode(ControlMode::CASCADE_CONTROL);
        }
    }
    
    // Compute control based on selected mode
    Vector4f control_cmd;
    
    switch (current_mode) {
        case ControlMode::EKF_STATE_FEEDBACK:
            control_cmd = compute_lqg_control();
            break;
            
        case ControlMode::ADAPTIVE_HYBRID:
            control_cmd = compute_adaptive_control();
            break;
            
        case ControlMode::CASCADE_CONTROL:
        default:
            // Use traditional cascade controller
            if (cascade_controller) {
                cascade_controller->set_position_target(position_cmd, attitude_cmd.z);
                cascade_controller->update_control_loop();
                
                // Get control outputs from cascade controller
                auto cascade_diag = cascade_controller->get_diagnostics();
                // Convert cascade outputs to our format
                control_cmd = Vector4f(0, 0, 0, 0.5f);  // Simplified for now
            }
            break;
    }
    
    // Apply safety limits
    control_output = control_cmd;
    apply_control_limits();
    
    // Update diagnostics
    update_diagnostics();
    
    return control_output;
}

bool EKFStateController::extract_ekf_state_vector()
{
    if (!ekf3 || !ekf3->healthy()) {
        return false;
    }
    
    // Extract position
    Vector2f pos_ne;
    float pos_d;
    if (!ekf3->getPosNE(pos_ne) || !ekf3->getPosD(pos_d)) {
        return false;
    }
    
    state_estimate(EKFStateIndices::PN - EKFStateIndices::POS_START) = pos_ne.x;
    state_estimate(EKFStateIndices::PE - EKFStateIndices::POS_START) = pos_ne.y;
    state_estimate(EKFStateIndices::PD - EKFStateIndices::POS_START) = pos_d;
    
    // Extract velocity
    Vector3f vel_ned;
    if (!ekf3->getVelNED(vel_ned)) {
        return false;
    }
    
    state_estimate(3) = vel_ned.x;  // VN
    state_estimate(4) = vel_ned.y;  // VE
    state_estimate(5) = vel_ned.z;  // VD
    
    // Extract attitude (convert quaternion to Euler)
    Vector3f euler_angles;
    if (!ekf3->getEulerAngles(euler_angles)) {
        return false;
    }
    
    state_estimate(6) = euler_angles.x;  // Roll
    state_estimate(7) = euler_angles.y;  // Pitch
    state_estimate(8) = euler_angles.z;  // Yaw
    
    // Extract angular rates
    Vector3f gyro_corrected;
    if (!ekf3->getGyroBias(gyro_corrected)) {
        // Use raw gyro if bias-corrected not available
        gyro_corrected = ahrs->get_gyro();
    }
    
    state_estimate(9) = gyro_corrected.x;   // Roll rate
    state_estimate(10) = gyro_corrected.y;  // Pitch rate
    state_estimate(11) = gyro_corrected.z;  // Yaw rate
    
    return true;
}

Vector4f EKFStateController::compute_lqg_control()
{
    // Compute state error
    compute_state_error();
    
    // Update adaptive gains based on EKF uncertainty
    update_adaptive_gains();
    
    // LQR control law: u = -K * x_error
    Vector4f control_effort = K_adaptive * state_error;
    
    // Add wind compensation if enabled
    if (wind_compensation_enabled) {
        Vector3f wind_compensation = compute_wind_compensation();
        control_effort.x += wind_compensation.x;  // Roll compensation
        control_effort.y += wind_compensation.y;  // Pitch compensation
    }
    
    // Add reference feedforward
    Vector4f feedforward = compute_reference_feedforward();
    control_effort += feedforward;
    
    return control_effort;
}

Vector4f EKFStateController::compute_adaptive_control()
{
    // Compute base LQG control
    Vector4f base_control = compute_lqg_control();
    
    // Monitor estimation uncertainty
    compute_estimation_uncertainty();
    
    // Switch between control modes based on uncertainty
    if (current_uncertainty > lqg_config.uncertainty_threshold) {
        // High uncertainty - blend with cascade control
        float blend_factor = constrain_float(
            (current_uncertainty - lqg_config.uncertainty_threshold) / lqg_config.uncertainty_threshold,
            0.0f, 1.0f
        );
        
        // Get cascade control output
        Vector4f cascade_control = Vector4f(0, 0, 0, 0.5f);  // Simplified
        
        // Blend controls
        return base_control * (1.0f - blend_factor) + cascade_control * blend_factor;
    }
    
    return base_control;
}

void EKFStateController::compute_state_error()
{
    for (int i = 0; i < 12; i++) {
        state_error(i) = state_reference(i) - state_estimate(i);
    }
    
    // Handle angle wrapping for yaw
    state_error(8) = wrap_PI(state_error(8));
}

void EKFStateController::update_adaptive_gains()
{
    // Get EKF covariance for adaptation
    float pos_var, vel_var, hgt_var, tas_var;
    Vector3f mag_var;
    Vector2f offset;
    
    if (ekf3->getVariances(vel_var, pos_var, hgt_var, mag_var, tas_var, offset)) {
        // Compute total uncertainty metric
        current_uncertainty = sqrtf(pos_var + vel_var + hgt_var);
        
        // Adapt gains based on uncertainty
        adaptive_gain_factor = constrain_float(
            lqg_config.max_gain_factor / (1.0f + current_uncertainty * lqg_config.adaptation_rate),
            lqg_config.min_gain_factor,
            lqg_config.max_gain_factor
        );
        
        // Scale LQR gains
        K_adaptive = K_lqr * adaptive_gain_factor;
    } else {
        // Use nominal gains if variance not available
        K_adaptive = K_lqr;
        adaptive_gain_factor = 1.0f;
    }
}

void EKFStateController::compute_estimation_uncertainty()
{
    // Extract innovation sequence from EKF
    Vector3f vel_innovations, pos_innovations, mag_innovations;
    float tas_innovation, yaw_innovation;
    
    if (ekf3->getInnovations(vel_innovations, pos_innovations, mag_innovations, tas_innovation, yaw_innovation)) {
        // Compute innovation magnitude
        float innovation_magnitude = vel_innovations.length() + pos_innovations.length();
        
        // Update uncertainty estimate with low-pass filter
        float alpha = 0.1f;  // Filter coefficient
        current_uncertainty = (1.0f - alpha) * current_uncertainty + alpha * innovation_magnitude;
        
        // Store for diagnostics
        diagnostics.innovation_sequence = Vector4f(
            pos_innovations.x, pos_innovations.y, vel_innovations.x, vel_innovations.y
        );
    }
}

Vector4f EKFStateController::compute_reference_feedforward()
{
    // Compute feedforward based on reference acceleration
    Vector3f ref_accel = Vector3f(0, 0, 0);  // Simplified - would compute from reference trajectory
    
    // Convert desired acceleration to control inputs
    Vector4f feedforward;
    
    // Roll/pitch for horizontal acceleration
    feedforward.x = ref_accel.y * system_id.mass / control_limits.x;  // Roll torque
    feedforward.y = -ref_accel.x * system_id.mass / control_limits.y; // Pitch torque
    feedforward.z = 0;  // Yaw torque
    
    // Thrust for vertical acceleration
    feedforward.w = (system_id.mass * (GRAVITY_MSS - ref_accel.z)) / control_limits.w;
    
    return feedforward;
}

Vector3f EKFStateController::compute_wind_compensation()
{
    // Update wind estimate from EKF
    update_wind_estimate();
    
    // Compute compensation torques to reject wind disturbance
    Vector3f wind_torques;
    
    // Wind creates apparent roll/pitch angles - compensate
    wind_torques.x = -wind_estimate.y * disturbance_rejection_gains.x;  // Roll compensation
    wind_torques.y = wind_estimate.x * disturbance_rejection_gains.y;   // Pitch compensation
    wind_torques.z = 0;  // No direct yaw compensation from horizontal wind
    
    return wind_torques;
}

void EKFStateController::update_wind_estimate()
{
    // Get wind estimate from EKF if available
    Vector3f wind_vel;
    if (ekf3->getWind(wind_vel)) {
        wind_estimate = wind_vel;
    }
}

void EKFStateController::apply_control_limits()
{
    // Saturate control outputs to physical limits
    control_output.x = constrain_float(control_output.x, -control_limits.x, control_limits.x);
    control_output.y = constrain_float(control_output.y, -control_limits.y, control_limits.y);
    control_output.z = constrain_float(control_output.z, -control_limits.z, control_limits.z);
    control_output.w = constrain_float(control_output.w, 0.1f, control_limits.w);
}

Matrix<float, 12, 12> EKFStateController::compute_lqr_cost_matrix()
{
    // Build Q matrix from configuration
    Q_matrix.zero();
    
    // Position weights [0:2]
    Q_matrix(0, 0) = lqg_config.position_weights.x;
    Q_matrix(1, 1) = lqg_config.position_weights.y;
    Q_matrix(2, 2) = lqg_config.position_weights.z;
    
    // Velocity weights [3:5]
    Q_matrix(3, 3) = lqg_config.velocity_weights.x;
    Q_matrix(4, 4) = lqg_config.velocity_weights.y;
    Q_matrix(5, 5) = lqg_config.velocity_weights.z;
    
    // Attitude weights [6:8]
    Q_matrix(6, 6) = lqg_config.attitude_weights.x;
    Q_matrix(7, 7) = lqg_config.attitude_weights.y;
    Q_matrix(8, 8) = lqg_config.attitude_weights.z;
    
    // Angular rate weights [9:11]
    Q_matrix(9, 9) = lqg_config.angular_rate_weights.x;
    Q_matrix(10, 10) = lqg_config.angular_rate_weights.y;
    Q_matrix(11, 11) = lqg_config.angular_rate_weights.z;
    
    return Q_matrix;
}

Matrix<float, 4, 4> EKFStateController::compute_control_cost_matrix()
{
    // Build R matrix from configuration
    R_matrix.zero();
    
    R_matrix(0, 0) = lqg_config.control_weights.x;  // Roll torque cost
    R_matrix(1, 1) = lqg_config.control_weights.y;  // Pitch torque cost
    R_matrix(2, 2) = lqg_config.control_weights.z;  // Yaw torque cost
    R_matrix(3, 3) = lqg_config.control_weights.w;  // Thrust cost
    
    return R_matrix;
}

void EKFStateController::compute_initial_lqr_gains()
{
    // Simplified LQR gain computation for initialization
    // In practice, this would solve the Riccati equation
    
    K_lqr.zero();
    
    // Position to thrust mapping (simplified)
    K_lqr(3, 2) = 5.0f;  // Vertical position to thrust
    
    // Velocity to attitude mapping
    K_lqr(0, 4) = -2.0f; // East velocity to roll torque
    K_lqr(1, 3) = 2.0f;  // North velocity to pitch torque
    
    // Attitude to torque mapping
    K_lqr(0, 6) = 10.0f; // Roll angle to roll torque
    K_lqr(1, 7) = 10.0f; // Pitch angle to pitch torque
    K_lqr(2, 8) = 5.0f;  // Yaw angle to yaw torque
    
    // Angular rate to torque mapping
    K_lqr(0, 9) = 1.0f;  // Roll rate to roll torque
    K_lqr(1, 10) = 1.0f; // Pitch rate to pitch torque
    K_lqr(2, 11) = 0.5f; // Yaw rate to yaw torque
    
    // Initialize adaptive gains
    K_adaptive = K_lqr;
}

void EKFStateController::set_control_mode(ControlMode mode)
{
    if (mode != current_mode) {
        uint32_t current_time = AP_HAL::millis();
        
        // Check hysteresis
        if (current_time - last_mode_switch_time_ms > mode_switch_hysteresis) {
            hal.console->printf("EKFStateController: Switching from mode %d to mode %d\n", 
                              (int)current_mode, (int)mode);
            
            current_mode = mode;
            mode_switch_count++;
            last_mode_switch_time_ms = current_time;
            
            // Reset integrators when switching modes
            if (cascade_controller) {
                // Reset cascade controller integrators
            }
            
            diagnostics.mode_switch_count = mode_switch_count;
        }
    }
    
    diagnostics.active_mode = current_mode;
}

void EKFStateController::set_position_reference(const Vector3f& pos_ref, const Vector3f& vel_ref)
{
    // Set position reference [0:2]
    state_reference(0) = pos_ref.x;
    state_reference(1) = pos_ref.y;
    state_reference(2) = pos_ref.z;
    
    // Set velocity reference [3:5]
    state_reference(3) = vel_ref.x;
    state_reference(4) = vel_ref.y;
    state_reference(5) = vel_ref.z;
    
    // Zero attitude and rate references for position control [6:11]
    for (int i = 6; i < 12; i++) {
        state_reference(i) = 0.0f;
    }
}

void EKFStateController::update_diagnostics()
{
    diagnostics.active_mode = current_mode;
    diagnostics.estimation_uncertainty = current_uncertainty;
    diagnostics.adaptive_gain_factor = adaptive_gain_factor;
    diagnostics.control_effort_magnitude = compute_control_effort_magnitude();
    diagnostics.ekf_healthy = (ekf3 && ekf3->healthy());
    
    // Compute errors
    diagnostics.position_error = Vector3f(state_error(0), state_error(1), state_error(2));
    diagnostics.velocity_error = Vector3f(state_error(3), state_error(4), state_error(5));
    diagnostics.attitude_error = Vector3f(state_error(6), state_error(7), state_error(8));
}

float EKFStateController::compute_control_effort_magnitude() const
{
    return sqrtf(control_output.x*control_output.x + 
                control_output.y*control_output.y + 
                control_output.z*control_output.z + 
                control_output.w*control_output.w);
}

// Getter methods
Vector3f EKFStateController::get_position_estimate() const
{
    return Vector3f(state_estimate(0), state_estimate(1), state_estimate(2));
}

Vector3f EKFStateController::get_velocity_estimate() const
{
    return Vector3f(state_estimate(3), state_estimate(4), state_estimate(5));
}

Vector3f EKFStateController::get_attitude_estimate() const
{
    return Vector3f(state_estimate(6), state_estimate(7), state_estimate(8));
}

EKFStateController::ControlDiagnostics EKFStateController::get_diagnostics() const
{
    return diagnostics;
}

// Configuration methods
void EKFStateController::set_lqg_configuration(const LQGConfiguration& config)
{
    lqg_config = config;
    
    // Recompute matrices with new configuration
    compute_lqr_cost_matrix();
    compute_control_cost_matrix();
    compute_initial_lqr_gains();
}

void EKFStateController::enable_wind_compensation(bool enable)
{
    wind_compensation_enabled = enable;
    if (enable) {
        hal.console->printf("EKFStateController: Wind compensation enabled\n");
    }
}

void EKFStateController::set_disturbance_rejection_gains(const Vector3f& gains)
{
    disturbance_rejection_gains = gains;
}

// ============================================================================
// ControlModeManager Implementation
// ============================================================================

ControlModeManager::ControlModeManager(EKFStateController* ekf_controller, HexacopterControlSystem* cascade_controller) :
    ekf_ctrl(ekf_controller),
    cascade_ctrl(cascade_controller),
    active_mode(ControlMode::CASCADE_CONTROL),
    requested_mode(ControlMode::CASCADE_CONTROL),
    uncertainty_threshold(0.5f),
    performance_threshold(0.8f),
    stability_margin_threshold(0.3f),
    switching_state(SwitchingState::STABLE)
{
}

void ControlModeManager::update_mode_selection(float dt)
{
    if (!ekf_ctrl || !cascade_ctrl) {
        return;
    }
    
    // Get current performance metrics
    auto ekf_diagnostics = ekf_ctrl->get_diagnostics();
    
    // Evaluate switching criteria
    bool ekf_healthy = ekf_diagnostics.ekf_healthy;
    bool uncertainty_ok = ekf_diagnostics.estimation_uncertainty < uncertainty_threshold;
    bool performance_ok = true;  // Would implement performance evaluation
    
    // State machine for mode switching
    switch (switching_state) {
        case SwitchingState::STABLE:
            if (active_mode == ControlMode::CASCADE_CONTROL && ekf_healthy && uncertainty_ok) {
                // Consider switching to EKF feedback
                switching_state = SwitchingState::EVALUATING;
                evaluation_start_time = AP_HAL::millis();
            } else if (active_mode == ControlMode::EKF_STATE_FEEDBACK && (!ekf_healthy || !uncertainty_ok)) {
                // Consider switching back to cascade
                switching_state = SwitchingState::EVALUATING;
                evaluation_start_time = AP_HAL::millis();
            }
            break;
            
        case SwitchingState::EVALUATING:
            // Evaluate for 2 seconds before switching
            if (AP_HAL::millis() - evaluation_start_time > 2000) {
                if (safe_to_switch_mode()) {
                    switching_state = SwitchingState::SWITCHING;
                    switching_start_time = AP_HAL::millis();
                } else {
                    switching_state = SwitchingState::STABLE;
                }
            }
            break;
            
        case SwitchingState::SWITCHING:
            perform_switching_sequence();
            switching_state = SwitchingState::MONITORING;
            break;
            
        case SwitchingState::MONITORING:
            // Monitor for 5 seconds after switch
            if (AP_HAL::millis() - switching_start_time > 5000) {
                switching_state = SwitchingState::STABLE;
            }
            break;
    }
}

bool ControlModeManager::safe_to_switch_mode()
{
    // Implement safety checks for mode switching
    // Check attitude limits, velocity limits, EKF health, etc.
    
    auto ekf_diag = ekf_ctrl->get_diagnostics();
    
    // Basic safety checks
    bool attitude_safe = (fabsf(ekf_diag.attitude_error.x) < radians(30) && 
                         fabsf(ekf_diag.attitude_error.y) < radians(30));
    bool velocity_safe = ekf_diag.velocity_error.length() < 5.0f;
    bool ekf_healthy = ekf_diag.ekf_healthy;
    
    return attitude_safe && velocity_safe && ekf_healthy;
}

void ControlModeManager::perform_switching_sequence()
{
    if (active_mode == ControlMode::CASCADE_CONTROL) {
        // Switch to EKF feedback
        ekf_ctrl->set_control_mode(ControlMode::EKF_STATE_FEEDBACK);
        active_mode = ControlMode::EKF_STATE_FEEDBACK;
        hal.console->printf("ControlModeManager: Switched to EKF State Feedback\n");
    } else {
        // Switch to cascade
        ekf_ctrl->set_control_mode(ControlMode::CASCADE_CONTROL);
        active_mode = ControlMode::CASCADE_CONTROL;
        hal.console->printf("ControlModeManager: Switched to Cascade Control\n");
    }
}

ControlMode ControlModeManager::get_active_mode() const
{
    return active_mode;
}

// Factory functions
EKFStateController* create_ekf_state_controller(const HexacopterSystemID& system_id)
{
    return new EKFStateController(system_id);
}

ControlModeManager* create_mode_manager(EKFStateController* ekf_ctrl, HexacopterControlSystem* cascade_ctrl)
{
    return new ControlModeManager(ekf_ctrl, cascade_ctrl);
}