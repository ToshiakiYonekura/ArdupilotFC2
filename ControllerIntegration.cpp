/*
 * Controller Integration Example
 * 
 * This file demonstrates how to integrate both the traditional cascade controller
 * and the new EKF state feedback controller with automatic mode switching.
 * 
 * Shows practical implementation for ArduCopter integration.
 */

#include "HexacopterController.h"
#include "EKF_StateController.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

// ============================================================================
// Integrated Hexacopter Control System
// ============================================================================

class IntegratedHexacopterController {
public:
    // Constructor
    IntegratedHexacopterController();
    
    // Initialization
    bool initialize();
    
    // Main control update (called from ArduCopter main loop)
    void update_control(const Vector3f& position_cmd, 
                       const Vector3f& velocity_cmd,
                       const Vector3f& attitude_cmd,
                       float yaw_rate_cmd);
    
    // Mode switching interface
    void set_control_mode(ControlMode mode);
    ControlMode get_active_control_mode() const;
    
    // Performance monitoring
    void get_control_status(struct ControlStatus& status);
    
    // Parameter interface
    void update_parameters();
    
private:
    // System identification parameters
    HexacopterSystemID system_id;
    
    // Controller instances
    HexacopterControlSystem* cascade_controller;
    EKFStateController* ekf_state_controller;
    ControlModeManager* mode_manager;
    
    // Configuration parameters
    AP_Int8 control_mode_param;
    AP_Int8 auto_switching_enabled;
    AP_Float uncertainty_threshold;
    AP_Float performance_threshold;
    
    // Control outputs
    Vector4f final_control_output;
    
    // Status and diagnostics
    struct ControlStatus {
        ControlMode active_mode;
        bool ekf_healthy;
        float estimation_uncertainty;
        float control_performance;
        Vector3f position_error;
        Vector3f velocity_error;
        uint32_t mode_switches;
        bool controllers_healthy;
    } control_status;
    
    // Internal methods
    void update_system_identification();
    void monitor_performance();
    void handle_mode_switching();
    bool validate_control_output(const Vector4f& control_cmd);
    void apply_final_safety_limits();
};

// Parameter definitions for ArduPilot integration
const AP_Param::GroupInfo IntegratedHexacopterController::var_info[] = {
    // Control mode selection
    AP_GROUPINFO("MODE", 1, IntegratedHexacopterController, control_mode_param, 0),
    AP_GROUPINFO("AUTO_SWITCH", 2, IntegratedHexacopterController, auto_switching_enabled, 1),
    AP_GROUPINFO("UNCERT_THRESH", 3, IntegratedHexacopterController, uncertainty_threshold, 0.5f),
    AP_GROUPINFO("PERF_THRESH", 4, IntegratedHexacopterController, performance_threshold, 0.8f),
    AP_GROUPEND
};

IntegratedHexacopterController::IntegratedHexacopterController() :
    cascade_controller(nullptr),
    ekf_state_controller(nullptr),
    mode_manager(nullptr),
    control_mode_param(0),
    auto_switching_enabled(1),
    uncertainty_threshold(0.5f),
    performance_threshold(0.8f)
{
    // Initialize system ID with default values
    system_id = HexacopterSystemID();
    
    // Initialize control status
    control_status.active_mode = ControlMode::CASCADE_CONTROL;
    control_status.ekf_healthy = false;
    control_status.estimation_uncertainty = 0.0f;
    control_status.control_performance = 0.0f;
    control_status.mode_switches = 0;
    control_status.controllers_healthy = false;
}

bool IntegratedHexacopterController::initialize()
{
    hal.console->printf("IntegratedHexacopterController: Initializing...\n");
    
    // Update system identification parameters from stored values
    update_system_identification();
    
    // Initialize cascade controller
    cascade_controller = new HexacopterControlSystem();
    if (!cascade_controller || !cascade_controller->initialize()) {
        hal.console->printf("IntegratedHexacopterController: Failed to initialize cascade controller\n");
        return false;
    }
    
    // Initialize EKF state feedback controller
    ekf_state_controller = create_ekf_state_controller(system_id);
    if (!ekf_state_controller || !ekf_state_controller->initialize()) {
        hal.console->printf("IntegratedHexacopterController: Failed to initialize EKF controller\n");
        return false;
    }
    
    // Initialize mode manager
    mode_manager = create_mode_manager(ekf_state_controller, cascade_controller);
    if (!mode_manager) {
        hal.console->printf("IntegratedHexacopterController: Failed to initialize mode manager\n");
        return false;
    }
    
    // Set initial control mode
    ControlMode initial_mode = static_cast<ControlMode>(control_mode_param.get());
    set_control_mode(initial_mode);
    
    // Configure LQG parameters from stored values
    LQGConfiguration lqg_config;
    // Would load from parameters here
    ekf_state_controller->set_lqg_configuration(lqg_config);
    
    control_status.controllers_healthy = true;
    
    hal.console->printf("IntegratedHexacopterController: Initialization complete\n");
    return true;
}

void IntegratedHexacopterController::update_control(const Vector3f& position_cmd, 
                                                   const Vector3f& velocity_cmd,
                                                   const Vector3f& attitude_cmd,
                                                   float yaw_rate_cmd)
{
    // Update parameters if changed
    update_parameters();
    
    // Handle automatic mode switching if enabled
    if (auto_switching_enabled.get()) {
        handle_mode_switching();
    }
    
    // Get current active mode
    ControlMode active_mode = mode_manager->get_active_mode();
    
    // Compute control based on active mode
    Vector4f control_output;
    
    switch (active_mode) {
        case ControlMode::CASCADE_CONTROL:
            // Use traditional cascade controller
            cascade_controller->set_position_target(position_cmd, attitude_cmd.z);
            cascade_controller->update_control_loop();
            
            // Extract control outputs (simplified)
            control_output = Vector4f(0, 0, 0, 0.5f);  // Would extract from cascade controller
            break;
            
        case ControlMode::EKF_STATE_FEEDBACK:
        case ControlMode::ADAPTIVE_HYBRID:
            // Use EKF state feedback controller
            control_output = ekf_state_controller->update_control(
                position_cmd, velocity_cmd, attitude_cmd, yaw_rate_cmd, 0.0025f  // 400Hz = 2.5ms
            );
            break;
    }
    
    // Validate and apply safety limits
    if (validate_control_output(control_output)) {
        final_control_output = control_output;
        apply_final_safety_limits();
    } else {
        // Fallback to safe hover if control output invalid
        final_control_output = Vector4f(0, 0, 0, 0.35f);  // Hover throttle
        hal.console->printf("IntegratedHexacopterController: Invalid control output, using safe hover\n");
    }
    
    // Update performance monitoring
    monitor_performance();
    
    // Update control status
    control_status.active_mode = active_mode;
    if (ekf_state_controller) {
        auto ekf_diag = ekf_state_controller->get_diagnostics();
        control_status.ekf_healthy = ekf_diag.ekf_healthy;
        control_status.estimation_uncertainty = ekf_diag.estimation_uncertainty;
        control_status.position_error = ekf_diag.position_error;
        control_status.velocity_error = ekf_diag.velocity_error;
        control_status.mode_switches = ekf_diag.mode_switch_count;
    }
}

void IntegratedHexacopterController::handle_mode_switching()
{
    if (!mode_manager || !ekf_state_controller) {
        return;
    }
    
    // Update mode manager with current conditions
    mode_manager->update_mode_selection(0.0025f);  // 400Hz update rate
    
    // Configure switching criteria from parameters
    mode_manager->set_switching_criteria(
        uncertainty_threshold.get(),
        performance_threshold.get(),
        0.3f  // stability margin
    );
}

bool IntegratedHexacopterController::validate_control_output(const Vector4f& control_cmd)
{
    // Check for NaN or infinite values
    if (!is_finite(control_cmd.x) || !is_finite(control_cmd.y) || 
        !is_finite(control_cmd.z) || !is_finite(control_cmd.w)) {
        return false;
    }
    
    // Check control magnitude limits
    float torque_magnitude = sqrtf(control_cmd.x*control_cmd.x + 
                                  control_cmd.y*control_cmd.y + 
                                  control_cmd.z*control_cmd.z);
    
    if (torque_magnitude > 20.0f) {  // Reasonable torque limit
        return false;
    }
    
    // Check thrust limits
    if (control_cmd.w < 0.0f || control_cmd.w > 1.0f) {
        return false;
    }
    
    return true;
}

void IntegratedHexacopterController::apply_final_safety_limits()
{
    // Final safety limits before sending to motors
    final_control_output.x = constrain_float(final_control_output.x, -15.0f, 15.0f);  // Roll torque
    final_control_output.y = constrain_float(final_control_output.y, -15.0f, 15.0f);  // Pitch torque
    final_control_output.z = constrain_float(final_control_output.z, -5.0f, 5.0f);    // Yaw torque
    final_control_output.w = constrain_float(final_control_output.w, 0.1f, 1.0f);     // Thrust
}

void IntegratedHexacopterController::monitor_performance()
{
    // Implement performance monitoring
    // Calculate tracking errors, control effort, etc.
    
    if (ekf_state_controller) {
        auto ekf_diag = ekf_state_controller->get_diagnostics();
        
        // Simple performance metric based on tracking errors
        float position_error_magnitude = ekf_diag.position_error.length();
        float velocity_error_magnitude = ekf_diag.velocity_error.length();
        
        // Combine errors into performance score (0-1, higher is better)
        control_status.control_performance = 1.0f / (1.0f + position_error_magnitude + velocity_error_magnitude);
    }
}

void IntegratedHexacopterController::update_system_identification()
{
    // Update system ID parameters from experimental results
    // This would typically load from EEPROM or parameter storage
    
    // Use identified values if available, otherwise use defaults
    system_id.mass = 2.5f;                    // From experimental measurement
    system_id.inertia.x = 0.0347f;           // From bifilar pendulum test
    system_id.inertia.y = 0.0347f;           // From bifilar pendulum test
    system_id.inertia.z = 0.0617f;           // From bifilar pendulum test
    system_id.arm_length = 0.25f;            // From vehicle geometry
    system_id.thrust_coeff = 3.16e-6f;       // From motor testing
    system_id.torque_coeff = 7.94e-8f;       // From motor testing
    system_id.motor_time_constant = 0.15f;   // From step response testing
    system_id.max_thrust_per_motor = 15.0f;  // From motor testing
    system_id.drag_coeff = Vector3f(0.25f, 0.25f, 0.30f);  // From flight testing
}

void IntegratedHexacopterController::update_parameters()
{
    // Check if parameters have changed and update controllers accordingly
    static int8_t last_control_mode = -1;
    
    if (control_mode_param.get() != last_control_mode) {
        ControlMode new_mode = static_cast<ControlMode>(control_mode_param.get());
        set_control_mode(new_mode);
        last_control_mode = control_mode_param.get();
    }
}

void IntegratedHexacopterController::set_control_mode(ControlMode mode)
{
    if (ekf_state_controller) {
        ekf_state_controller->set_control_mode(mode);
    }
    
    hal.console->printf("IntegratedHexacopterController: Control mode set to %d\n", (int)mode);
}

ControlMode IntegratedHexacopterController::get_active_control_mode() const
{
    if (mode_manager) {
        return mode_manager->get_active_mode();
    }
    return ControlMode::CASCADE_CONTROL;
}

void IntegratedHexacopterController::get_control_status(struct ControlStatus& status)
{
    status = control_status;
}

// ============================================================================
// ArduCopter Integration Example
// ============================================================================

// Example of how to integrate with ArduCopter's main control loop
class ArduCopterIntegration {
private:
    IntegratedHexacopterController* integrated_controller;
    
public:
    bool init() {
        // Initialize integrated controller
        integrated_controller = new IntegratedHexacopterController();
        
        if (!integrated_controller || !integrated_controller->initialize()) {
            hal.console->printf("ArduCopter: Failed to initialize integrated controller\n");
            return false;
        }
        
        hal.console->printf("ArduCopter: Integrated hexacopter controller initialized\n");
        return true;
    }
    
    void update_flight_mode_stabilize() {
        // Called from Copter::Mode::Stabilize::run()
        
        // Get pilot inputs
        Vector3f pilot_input = get_pilot_input();  // Would implement
        
        // Convert to attitude commands
        Vector3f attitude_cmd = convert_pilot_to_attitude(pilot_input);
        
        // Call integrated controller with attitude commands
        integrated_controller->update_control(
            Vector3f(),        // No position command in stabilize
            Vector3f(),        // No velocity command in stabilize
            attitude_cmd,      // Attitude from pilot
            pilot_input.z      // Yaw rate from pilot
        );
    }
    
    void update_flight_mode_guided() {
        // Called from Copter::Mode::Guided::run()
        
        // Get guidance commands from GCS or companion computer
        Vector3f position_cmd = get_guided_position_target();  // Would implement
        Vector3f velocity_cmd = get_guided_velocity_target();  // Would implement
        
        // Call integrated controller with position/velocity commands
        integrated_controller->update_control(
            position_cmd,
            velocity_cmd,
            Vector3f(),        // Attitude computed by controller
            0.0f               // Yaw rate computed by controller
        );
    }
    
    void update_flight_mode_auto() {
        // Called from Copter::Mode::Auto::run()
        
        // Get waypoint commands from mission
        Vector3f wp_position = get_current_waypoint_position();  // Would implement
        Vector3f wp_velocity = get_current_waypoint_velocity();  // Would implement
        
        // Call integrated controller
        integrated_controller->update_control(
            wp_position,
            wp_velocity,
            Vector3f(),        // Attitude computed by controller
            0.0f               // Yaw rate from mission or computed
        );
    }
    
    // Interface functions (would be implemented)
    Vector3f get_pilot_input() { return Vector3f(); }
    Vector3f convert_pilot_to_attitude(const Vector3f& pilot) { return Vector3f(); }
    Vector3f get_guided_position_target() { return Vector3f(); }
    Vector3f get_guided_velocity_target() { return Vector3f(); }
    Vector3f get_current_waypoint_position() { return Vector3f(); }
    Vector3f get_current_waypoint_velocity() { return Vector3f(); }
};

// Global instance for ArduCopter integration
static ArduCopterIntegration* g_hexacopter_integration = nullptr;

// Initialization function to be called from ArduCopter setup
extern "C" bool init_hexacopter_controllers() {
    g_hexacopter_integration = new ArduCopterIntegration();
    
    if (!g_hexacopter_integration || !g_hexacopter_integration->init()) {
        delete g_hexacopter_integration;
        g_hexacopter_integration = nullptr;
        return false;
    }
    
    return true;
}

// Update function to be called from ArduCopter main loop
extern "C" void update_hexacopter_controllers() {
    if (g_hexacopter_integration) {
        // Would call appropriate update function based on flight mode
        // g_hexacopter_integration->update_flight_mode_guided();
    }
}