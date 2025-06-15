/*
 * Hexacopter Control System Implementation
 * 
 * Implementation of comprehensive control system for 6-motor ArduCopter
 */

#include "HexacopterController.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// ============================================================================
// HexacopterMixer Implementation
// ============================================================================

HexacopterMixer::HexacopterMixer() :
    motor_min(0.15f),
    motor_max(0.95f),
    hover_throttle(0.35f)
{
}

bool HexacopterMixer::mix_controls(float thrust, const Vector3f& torque, float* motor_outputs)
{
    if (motor_outputs == nullptr) {
        return false;
    }
    
    // Apply mixing matrix to get individual motor thrusts
    for (int i = 0; i < 6; i++) {
        motor_outputs[i] = mixing_matrix[i][3] * thrust +           // Throttle
                          mixing_matrix[i][0] * torque.x +         // Roll
                          mixing_matrix[i][1] * torque.y +         // Pitch  
                          mixing_matrix[i][2] * torque.z;          // Yaw
    }
    
    // Apply motor limits
    apply_limits(motor_outputs);
    
    return true;
}

bool HexacopterMixer::check_saturation(const float* motor_outputs) const
{
    for (int i = 0; i < 6; i++) {
        if (motor_outputs[i] >= motor_max || motor_outputs[i] <= motor_min) {
            return true;
        }
    }
    return false;
}

void HexacopterMixer::apply_limits(float* motor_outputs) const
{
    for (int i = 0; i < 6; i++) {
        motor_outputs[i] = constrain_float(motor_outputs[i], motor_min, motor_max);
    }
}

// ============================================================================
// HexacopterAttitudeController Implementation  
// ============================================================================

HexacopterAttitudeController::HexacopterAttitudeController(const HexacopterSystemID& system_id) :
    control_mode(PID_MODE),
    sys_id(system_id),
    rate_p_gains(0.135f, 0.135f, 0.18f),
    rate_i_gains(0.135f, 0.135f, 0.018f),
    rate_d_gains(0.0036f, 0.0036f, 0.0f),
    attitude_p_gains(4.5f, 4.5f, 4.5f),
    integrator_limits(0.444f, 0.444f, 0.222f),
    filter_cutoff_freq(20.0f)
{
    reset_integrators();
}

Vector3f HexacopterAttitudeController::update(const Vector3f& attitude_error, 
                                             const Vector3f& angular_rate, 
                                             const Vector3f& angular_rate_cmd, 
                                             float dt)
{
    Vector3f rate_error = angular_rate_cmd - angular_rate;
    
    // Update based on control mode
    switch (control_mode) {
        case PID_MODE:
            rate_command = pid_attitude_control(attitude_error, dt);
            torque_command = pid_rate_control(rate_error, dt);
            break;
            
        case LQR_MODE:
            torque_command = lqr_control(attitude_error, rate_error, dt);
            break;
            
        case HYBRID_MODE:
            rate_command = pid_attitude_control(attitude_error, dt);
            torque_command = pid_rate_control(rate_error, dt);
            torque_command += compute_feedforward(angular_rate_cmd, dt);
            break;
    }
    
    // Apply motor dynamics compensation
    apply_motor_dynamics_compensation(torque_command);
    
    return torque_command;
}

Vector3f HexacopterAttitudeController::pid_attitude_control(const Vector3f& attitude_error, float dt)
{
    // Attitude P controller with angle limits
    Vector3f rate_cmd;
    rate_cmd.x = attitude_p_gains.x * constrain_float(attitude_error.x, -radians(45), radians(45));
    rate_cmd.y = attitude_p_gains.y * constrain_float(attitude_error.y, -radians(45), radians(45));
    rate_cmd.z = attitude_p_gains.z * attitude_error.z;
    
    // Apply rate limits
    rate_cmd.x = constrain_float(rate_cmd.x, -radians(220), radians(220));
    rate_cmd.y = constrain_float(rate_cmd.y, -radians(220), radians(220));
    rate_cmd.z = constrain_float(rate_cmd.z, -radians(200), radians(200));
    
    return rate_cmd;
}

Vector3f HexacopterAttitudeController::pid_rate_control(const Vector3f& rate_error, float dt)
{
    // Apply low-pass filter to rate error
    float alpha = dt / (dt + 1.0f / (2.0f * M_PI * filter_cutoff_freq));
    rate_error_filtered = rate_error_filtered * (1.0f - alpha) + rate_error * alpha;
    
    // PID calculation
    Vector3f p_term = rate_p_gains * rate_error;
    
    // Integrator with limits
    rate_integrator += rate_i_gains * rate_error * dt;
    rate_integrator.x = constrain_float(rate_integrator.x, -integrator_limits.x, integrator_limits.x);
    rate_integrator.y = constrain_float(rate_integrator.y, -integrator_limits.y, integrator_limits.y);
    rate_integrator.z = constrain_float(rate_integrator.z, -integrator_limits.z, integrator_limits.z);
    
    // Derivative term
    Vector3f d_term = rate_d_gains * (rate_error_filtered - last_rate_error) / dt;
    last_rate_error = rate_error_filtered;
    
    // Total torque command
    Vector3f torque_cmd = p_term + rate_integrator + d_term;
    
    // Apply torque limits based on motor capabilities
    float max_torque_roll = sys_id.max_thrust_per_motor * sys_id.arm_length * 6.0f * 0.5f;
    float max_torque_pitch = max_torque_roll;
    float max_torque_yaw = sys_id.max_thrust_per_motor * sys_id.torque_coeff / sys_id.thrust_coeff * 6.0f;
    
    torque_cmd.x = constrain_float(torque_cmd.x, -max_torque_roll, max_torque_roll);
    torque_cmd.y = constrain_float(torque_cmd.y, -max_torque_pitch, max_torque_pitch);
    torque_cmd.z = constrain_float(torque_cmd.z, -max_torque_yaw, max_torque_yaw);
    
    return torque_cmd;
}

Vector3f HexacopterAttitudeController::lqr_control(const Vector3f& attitude_error, 
                                                  const Vector3f& rate_error, float dt)
{
    // LQR control law: u = -K_att * attitude_error - K_rate * rate_error
    Vector3f torque_cmd = -(K_attitude * attitude_error + K_rate * rate_error);
    
    return torque_cmd;
}

Vector3f HexacopterAttitudeController::compute_feedforward(const Vector3f& angular_rate_cmd, float dt)
{
    // Feedforward based on desired angular acceleration
    static Vector3f last_rate_cmd;
    Vector3f angular_accel_cmd = (angular_rate_cmd - last_rate_cmd) / dt;
    last_rate_cmd = angular_rate_cmd;
    
    // Feedforward torque = Inertia * desired angular acceleration
    Vector3f ff_torque;
    ff_torque.x = sys_id.inertia.x * angular_accel_cmd.x;
    ff_torque.y = sys_id.inertia.y * angular_accel_cmd.y;
    ff_torque.z = sys_id.inertia.z * angular_accel_cmd.z;
    
    return ff_torque;
}

void HexacopterAttitudeController::apply_motor_dynamics_compensation(Vector3f& torque_cmd)
{
    // Compensate for motor time constants (inverse motor dynamics)
    float comp_factor = 1.0f + sys_id.motor_time_constant * filter_cutoff_freq;
    torque_cmd *= comp_factor;
}

void HexacopterAttitudeController::reset_integrators()
{
    rate_integrator.zero();
    last_rate_error.zero();
    rate_error_filtered.zero();
}

void HexacopterAttitudeController::set_rate_gains(const Vector3f& p_gains, 
                                                 const Vector3f& i_gains, 
                                                 const Vector3f& d_gains)
{
    rate_p_gains = p_gains;
    rate_i_gains = i_gains;
    rate_d_gains = d_gains;
}

void HexacopterAttitudeController::set_attitude_gains(const Vector3f& p_gains)
{
    attitude_p_gains = p_gains;
}

// ============================================================================
// HexacopterVelocityController Implementation
// ============================================================================

HexacopterVelocityController::HexacopterVelocityController(const HexacopterSystemID& system_id) :
    sys_id(system_id),
    pos_p_gains(1.0f, 1.0f, 1.0f),
    vel_p_gains(2.0f, 2.0f, 5.0f),
    vel_i_gains(1.0f, 1.0f, 2.0f),
    vel_d_gains(0.5f, 0.5f, 0.0f),
    accel_limits(5.0f, 5.0f, 4.0f),
    velocity_limits(10.0f, 10.0f, 5.0f),
    thrust_command(sys_id.mass * GRAVITY_MSS)
{
    velocity_integrator.zero();
    last_velocity_error.zero();
}

Vector3f HexacopterVelocityController::update_position_control(const Vector3f& position_error, float dt)
{
    // Position P controller
    velocity_command = pos_p_gains * position_error;
    
    // Apply velocity limits
    velocity_command = saturate_vector(velocity_command, velocity_limits);
    
    return velocity_command;
}

Vector3f HexacopterVelocityController::update_velocity_control(const Vector3f& velocity_error, 
                                                             const Vector3f& velocity_cmd, float dt)
{
    // Velocity PID controller
    Vector3f p_term = vel_p_gains * velocity_error;
    
    // Integrator with anti-windup
    velocity_integrator += vel_i_gains * velocity_error * dt;
    velocity_integrator = saturate_vector(velocity_integrator, Vector3f(1000.0f, 1000.0f, 800.0f));
    
    // Derivative term
    Vector3f d_term = vel_d_gains * (velocity_error - last_velocity_error) / dt;
    last_velocity_error = velocity_error;
    
    // Total acceleration command
    acceleration_command = p_term + velocity_integrator + d_term;
    
    // Apply acceleration limits
    acceleration_command = saturate_vector(acceleration_command, accel_limits);
    
    return acceleration_command;
}

void HexacopterVelocityController::velocity_to_attitude(const Vector3f& accel_desired, 
                                                       Vector3f& attitude_cmd, 
                                                       float& thrust_cmd)
{
    // Convert desired accelerations to attitude commands
    // Assuming small angle approximation and NED frame
    
    // Calculate required attitude for horizontal accelerations
    float pitch_cmd = asinf(constrain_float(accel_desired.x / GRAVITY_MSS, -0.5f, 0.5f));
    float roll_cmd = -asinf(constrain_float(accel_desired.y / (GRAVITY_MSS * cosf(pitch_cmd)), -0.5f, 0.5f));
    
    attitude_cmd.x = roll_cmd;
    attitude_cmd.y = pitch_cmd;
    // attitude_cmd.z (yaw) should be set by higher level controller
    
    // Calculate thrust command (vertical acceleration + gravity compensation)
    float desired_thrust_acceleration = GRAVITY_MSS - accel_desired.z;
    thrust_cmd = sys_id.mass * desired_thrust_acceleration / (cosf(roll_cmd) * cosf(pitch_cmd));
    
    // Normalize thrust command (0-1 range)
    thrust_cmd = constrain_float(thrust_cmd / (sys_id.mass * GRAVITY_MSS), 0.1f, 1.0f);
    
    thrust_command = thrust_cmd;
}

HexacopterVelocityController::TrajectoryPoint 
HexacopterVelocityController::generate_trajectory(const Vector3f& start_pos, 
                                                const Vector3f& end_pos, 
                                                float flight_time, 
                                                float current_time)
{
    TrajectoryPoint point;
    
    if (current_time >= flight_time) {
        point.position = end_pos;
        point.velocity.zero();
        point.acceleration.zero();
        return point;
    }
    
    // Compute trajectory coefficients if not already done
    compute_trajectory_coeffs(start_pos, end_pos, flight_time);
    
    // Evaluate 5th order polynomial
    float t = current_time;
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    
    point.position = trajectory_coeffs.a0 + trajectory_coeffs.a1 * t + 
                    trajectory_coeffs.a2 * t2 + trajectory_coeffs.a3 * t3 + 
                    trajectory_coeffs.a4 * t4 + trajectory_coeffs.a5 * t5;
    
    point.velocity = trajectory_coeffs.a1 + trajectory_coeffs.a2 * 2.0f * t +
                    trajectory_coeffs.a3 * 3.0f * t2 + trajectory_coeffs.a4 * 4.0f * t3 +
                    trajectory_coeffs.a5 * 5.0f * t4;
    
    point.acceleration = trajectory_coeffs.a2 * 2.0f + trajectory_coeffs.a3 * 6.0f * t +
                        trajectory_coeffs.a4 * 12.0f * t2 + trajectory_coeffs.a5 * 20.0f * t3;
    
    return point;
}

void HexacopterVelocityController::compute_trajectory_coeffs(const Vector3f& start_pos, 
                                                           const Vector3f& end_pos, float T)
{
    // 5th order polynomial with boundary conditions:
    // p(0) = start_pos, p'(0) = 0, p''(0) = 0
    // p(T) = end_pos, p'(T) = 0, p''(T) = 0
    
    Vector3f dp = end_pos - start_pos;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    
    trajectory_coeffs.a0 = start_pos;
    trajectory_coeffs.a1.zero();
    trajectory_coeffs.a2.zero();
    trajectory_coeffs.a3 = dp * (10.0f / T3);
    trajectory_coeffs.a4 = dp * (-15.0f / T4);
    trajectory_coeffs.a5 = dp * (6.0f / T5);
}

Vector3f HexacopterVelocityController::saturate_vector(const Vector3f& input, const Vector3f& limits)
{
    Vector3f output;
    output.x = constrain_float(input.x, -limits.x, limits.x);
    output.y = constrain_float(input.y, -limits.y, limits.y);
    output.z = constrain_float(input.z, -limits.z, limits.z);
    return output;
}

// ============================================================================
// HexacopterStateEstimator Implementation
// ============================================================================

HexacopterStateEstimator::HexacopterStateEstimator() :
    ekf(nullptr),
    last_healthy_time_ms(0),
    health_timeout_ms(1000.0f)
{
    last_innovations.zero();
    innovation_test_ratios.zero();
}

bool HexacopterStateEstimator::initialize(const HexacopterSystemID& system_id)
{
    // Get EKF3 instance
    ekf = AP::ahrs_navekf().get_NavEKF3_const();
    if (ekf == nullptr) {
        return false;
    }
    
    // EKF should already be initialized by ArduPilot
    // Here we just verify it's working correctly
    return is_healthy();
}

void HexacopterStateEstimator::update_imu(const Vector3f& gyro, const Vector3f& accel, float dt)
{
    // IMU data is automatically fed to EKF by ArduPilot's main loop
    // This function can be used for additional processing or logging
    check_ekf_health();
}

void HexacopterStateEstimator::update_gps(const Vector3f& position, const Vector3f& velocity, float accuracy)
{
    // GPS data is automatically fed to EKF by ArduPilot
    // Additional validation can be performed here
    if (validate_measurement(position, Vector3f(1000.0f, 1000.0f, 1000.0f)) &&
        validate_measurement(velocity, Vector3f(50.0f, 50.0f, 20.0f))) {
        // GPS data is valid
        monitor_innovations();
    }
}

Vector3f HexacopterStateEstimator::get_position() const
{
    Vector2f pos_ne;
    float pos_d;
    Vector3f position;
    
    if (ekf && ekf->getPosNE(pos_ne) && ekf->getPosD(pos_d)) {
        position.x = pos_ne.x;
        position.y = pos_ne.y;
        position.z = pos_d;
    }
    
    return position;
}

Vector3f HexacopterStateEstimator::get_velocity() const
{
    Vector3f velocity;
    if (ekf) {
        ekf->getVelNED(velocity);
    }
    return velocity;
}

Vector3f HexacopterStateEstimator::get_attitude() const
{
    Vector3f attitude;
    if (ekf) {
        ekf->getEulerAngles(attitude);
    }
    return attitude;
}

bool HexacopterStateEstimator::is_healthy() const
{
    if (!ekf) {
        return false;
    }
    
    return ekf->healthy() && (AP_HAL::millis() - last_healthy_time_ms < health_timeout_ms);
}

void HexacopterStateEstimator::check_ekf_health()
{
    if (ekf && ekf->healthy()) {
        last_healthy_time_ms = AP_HAL::millis();
    }
}

void HexacopterStateEstimator::monitor_innovations()
{
    if (!ekf) {
        return;
    }
    
    Vector3f vel_innov, pos_innov, mag_innov;
    float tas_innov, yaw_innov;
    
    if (ekf->getInnovations(vel_innov, pos_innov, mag_innov, tas_innov, yaw_innov)) {
        last_innovations = pos_innov;
        
        // Calculate innovation test ratios
        float vel_var, pos_var, hgt_var, tas_var;
        Vector3f mag_var;
        Vector2f offset;
        
        if (ekf->getVariances(vel_var, pos_var, hgt_var, mag_var, tas_var, offset)) {
            innovation_test_ratios.x = (pos_innov.x * pos_innov.x) / pos_var;
            innovation_test_ratios.y = (pos_innov.y * pos_innov.y) / pos_var;
            innovation_test_ratios.z = (pos_innov.z * pos_innov.z) / hgt_var;
        }
    }
}

// ============================================================================
// HexacopterControlSystem Implementation
// ============================================================================

HexacopterControlSystem::HexacopterControlSystem() :
    mixer(nullptr),
    attitude_controller(nullptr),
    velocity_controller(nullptr),
    state_estimator(nullptr),
    max_tilt_angle(radians(45)),
    max_climb_rate(5.0f),
    max_horizontal_speed(10.0f),
    last_update_time_us(0),
    position_control_counter(0),
    velocity_control_counter(0),
    emergency_stop_time_ms(0),
    system_healthy(false)
{
    position_target.zero();
    velocity_target.zero();
    attitude_target.zero();
    rate_target.zero();
    thrust_target = 0.0f;
    yaw_target = 0.0f;
}

bool HexacopterControlSystem::initialize()
{
    // Create control system components
    mixer = new HexacopterMixer();
    attitude_controller = new HexacopterAttitudeController(system_id);
    velocity_controller = new HexacopterVelocityController(system_id);
    state_estimator = new HexacopterStateEstimator();
    
    if (!mixer || !attitude_controller || !velocity_controller || !state_estimator) {
        return false;
    }
    
    // Initialize state estimator
    if (!state_estimator->initialize(system_id)) {
        return false;
    }
    
    system_healthy = true;
    last_update_time_us = AP_HAL::micros();
    
    return true;
}

void HexacopterControlSystem::update_control_loop()
{
    uint32_t current_time_us = AP_HAL::micros();
    float dt = (current_time_us - last_update_time_us) * 1e-6f;
    last_update_time_us = current_time_us;
    
    // Update diagnostics timing
    diagnostics.loop_time_us = current_time_us - last_update_time_us;
    
    // Update state estimation
    update_state_estimation();
    
    // Update control loops at different rates
    position_control_counter++;
    velocity_control_counter++;
    
    // Position control at 50Hz (every 8th cycle)
    if (position_control_counter % 8 == 0) {
        update_position_control();
    }
    
    // Velocity control at 100Hz (every 4th cycle)
    if (velocity_control_counter % 4 == 0) {
        update_velocity_control();
    }
    
    // Attitude control at 400Hz (every cycle)
    update_attitude_control();
    
    // Motor output
    update_motor_outputs();
    
    // Safety checks
    update_safety_checks();
    
    // Update diagnostics
    update_diagnostics();
}

void HexacopterControlSystem::update_state_estimation()
{
    // State estimation is handled automatically by ArduPilot
    // We just check health here
    system_healthy = state_estimator->is_healthy();
}

void HexacopterControlSystem::update_attitude_control()
{
    if (!system_healthy) {
        return;
    }
    
    // Get current states
    Vector3f current_attitude = state_estimator->get_attitude();
    Vector3f current_angular_rate = state_estimator->get_angular_rate();
    
    // Compute attitude error
    Vector3f attitude_error = attitude_target - current_attitude;
    
    // Wrap yaw error
    attitude_error.z = wrap_PI(attitude_error.z);
    
    // Update attitude controller
    float dt = 1.0f / 400.0f;  // 400Hz control loop
    Vector3f torque_cmd = attitude_controller->update(attitude_error, current_angular_rate, rate_target, dt);
    
    // Store for diagnostics
    diagnostics.attitude_error = attitude_error;
    diagnostics.rate_error = rate_target - current_angular_rate;
}

void HexacopterControlSystem::update_motor_outputs()
{
    if (!system_healthy || emergency_stop_time_ms > 0) {
        // Emergency stop - zero all motors
        for (int i = 0; i < 6; i++) {
            diagnostics.motor_outputs[i] = 0.0f;
        }
        return;
    }
    
    // Get torque command from attitude controller
    Vector3f torque_cmd = attitude_controller->get_torque_command();
    
    // Mix controls to motor outputs
    float motor_outputs[6];
    if (mixer->mix_controls(thrust_target, torque_cmd, motor_outputs)) {
        // Check for saturation
        if (mixer->check_saturation(motor_outputs)) {
            // Handle motor saturation - could implement prioritization here
        }
        
        // Store motor outputs for diagnostics
        for (int i = 0; i < 6; i++) {
            diagnostics.motor_outputs[i] = motor_outputs[i];
        }
        
        // Output to motors would be done here via AP_Motors interface
        // This is typically handled by ArduPilot's motor output system
    }
}

bool HexacopterControlSystem::is_system_healthy() const
{
    return system_healthy && state_estimator && state_estimator->is_healthy();
}

void HexacopterControlSystem::emergency_stop()
{
    emergency_stop_time_ms = AP_HAL::millis();
    thrust_target = 0.0f;
    attitude_target.zero();
    rate_target.zero();
    
    if (attitude_controller) {
        attitude_controller->reset_integrators();
    }
}

HexacopterControlSystem::ControlDiagnostics HexacopterControlSystem::get_diagnostics() const
{
    return diagnostics;
}

void HexacopterControlSystem::update_diagnostics()
{
    diagnostics.cpu_load_percent = calculate_cpu_load();
    
    // Update position and velocity errors if we have targets
    Vector3f current_position = state_estimator->get_position();
    Vector3f current_velocity = state_estimator->get_velocity();
    
    diagnostics.position_error = position_target - current_position;
    diagnostics.velocity_error = velocity_target - current_velocity;
}

float HexacopterControlSystem::calculate_cpu_load()
{
    // Simple CPU load calculation based on loop timing
    uint32_t expected_loop_time = 2500;  // 2.5ms for 400Hz
    float load = (float)diagnostics.loop_time_us / expected_loop_time * 100.0f;
    return constrain_float(load, 0.0f, 100.0f);
}