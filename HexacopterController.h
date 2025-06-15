/*
 * Hexacopter Control System Implementation
 * 
 * This file implements a comprehensive control system for a 6-motor ArduCopter
 * including attitude control, velocity control, and EKF integration.
 * 
 * Based on system identification and control design principles.
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_PosControl/AC_PosControl.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Motors/AP_MotorsMatrix.h>

// System identification parameters for hexacopter
struct HexacopterSystemID {
    // Physical parameters
    float mass;                    // Vehicle mass (kg)
    float arm_length;              // Arm length (m)
    Vector3f inertia;             // Moments of inertia [Ixx, Iyy, Izz] (kg⋅m²)
    
    // Motor parameters
    float thrust_coeff;            // Thrust coefficient kT (N⋅s²)
    float torque_coeff;            // Torque coefficient kM (N⋅m⋅s²)
    float motor_time_constant;     // Motor time constant (s)
    float max_thrust_per_motor;    // Maximum thrust per motor (N)
    
    // Drag coefficients
    Vector3f drag_coeff;          // Translational drag coefficients [Cdx, Cdy, Cdz]
    
    // Default constructor with typical hexacopter values
    HexacopterSystemID() :
        mass(2.5f),
        arm_length(0.25f),
        inertia(0.0347f, 0.0347f, 0.0617f),
        thrust_coeff(3.16e-6f),
        torque_coeff(7.94e-8f),
        motor_time_constant(0.15f),
        max_thrust_per_motor(15.0f),
        drag_coeff(0.25f, 0.25f, 0.30f)
    {}
};

// Motor mixing matrix for hexacopter X-configuration
class HexacopterMixer {
public:
    HexacopterMixer();
    
    // Mix control inputs to motor outputs
    bool mix_controls(float thrust, const Vector3f& torque, float* motor_outputs);
    
    // Check for motor saturation
    bool check_saturation(const float* motor_outputs) const;
    
    // Apply motor limits
    void apply_limits(float* motor_outputs) const;
    
private:
    // Mixing matrix: [roll, pitch, yaw, throttle] for each motor
    static constexpr float mixing_matrix[6][4] = {
        // Motor positions for X-configuration hexacopter
        { 0.50f,  0.866f,  1.0f, 1.0f},  // Motor 1: 30°,  CCW
        { 1.00f,  0.000f, -1.0f, 1.0f},  // Motor 2: 90°,  CW
        { 0.50f, -0.866f,  1.0f, 1.0f},  // Motor 3: 150°, CCW
        {-0.50f, -0.866f, -1.0f, 1.0f},  // Motor 4: 210°, CW
        {-1.00f,  0.000f,  1.0f, 1.0f},  // Motor 5: 270°, CCW
        {-0.50f,  0.866f, -1.0f, 1.0f}   // Motor 6: 330°, CW
    };
    
    float motor_min;
    float motor_max;
    float hover_throttle;
};

// Advanced attitude controller with multiple control strategies
class HexacopterAttitudeController {
public:
    enum ControlMode {
        PID_MODE,      // Standard PID control
        LQR_MODE,      // Linear Quadratic Regulator
        HYBRID_MODE    // Hybrid PID + feedforward
    };
    
    HexacopterAttitudeController(const HexacopterSystemID& system_id);
    
    // Main update function
    Vector3f update(const Vector3f& attitude_error, const Vector3f& angular_rate, 
                   const Vector3f& angular_rate_cmd, float dt);
    
    // Set control mode
    void set_control_mode(ControlMode mode) { control_mode = mode; }
    
    // Gain setting functions
    void set_rate_gains(const Vector3f& p_gains, const Vector3f& i_gains, const Vector3f& d_gains);
    void set_attitude_gains(const Vector3f& p_gains);
    void set_lqr_gains(const Matrix3f& K_attitude, const Matrix3f& K_rate);
    
    // Integrator management
    void reset_integrators();
    void set_integrator_limits(const Vector3f& limits);
    
    // Get controller outputs
    Vector3f get_rate_command() const { return rate_command; }
    Vector3f get_torque_command() const { return torque_command; }
    
private:
    // Control mode
    ControlMode control_mode;
    
    // System identification parameters
    HexacopterSystemID sys_id;
    
    // PID gains
    Vector3f rate_p_gains;
    Vector3f rate_i_gains;
    Vector3f rate_d_gains;
    Vector3f attitude_p_gains;
    
    // LQR gains
    Matrix3f K_attitude;
    Matrix3f K_rate;
    
    // Controller states
    Vector3f rate_integrator;
    Vector3f last_rate_error;
    Vector3f rate_command;
    Vector3f torque_command;
    Vector3f integrator_limits;
    
    // Filters
    Vector3f rate_error_filtered;
    float filter_cutoff_freq;
    
    // Internal controller methods
    Vector3f pid_attitude_control(const Vector3f& attitude_error, float dt);
    Vector3f pid_rate_control(const Vector3f& rate_error, float dt);
    Vector3f lqr_control(const Vector3f& attitude_error, const Vector3f& rate_error, float dt);
    Vector3f compute_feedforward(const Vector3f& angular_rate_cmd, float dt);
    void apply_motor_dynamics_compensation(Vector3f& torque_cmd);
};

// Velocity controller with EKF integration
class HexacopterVelocityController {
public:
    HexacopterVelocityController(const HexacopterSystemID& system_id);
    
    // Position control update (called at lower rate, e.g., 50Hz)
    Vector3f update_position_control(const Vector3f& position_error, float dt);
    
    // Velocity control update (called at higher rate, e.g., 100Hz)
    Vector3f update_velocity_control(const Vector3f& velocity_error, const Vector3f& velocity_cmd, float dt);
    
    // Get desired attitude and thrust from velocity commands
    void velocity_to_attitude(const Vector3f& accel_desired, Vector3f& attitude_cmd, float& thrust_cmd);
    
    // Trajectory generation
    struct TrajectoryPoint {
        Vector3f position;
        Vector3f velocity;
        Vector3f acceleration;
        float yaw;
        float yaw_rate;
    };
    
    TrajectoryPoint generate_trajectory(const Vector3f& start_pos, const Vector3f& end_pos, 
                                      float flight_time, float current_time);
    
    // Gain setting
    void set_position_gains(const Vector3f& p_gains);
    void set_velocity_gains(const Vector3f& p_gains, const Vector3f& i_gains, const Vector3f& d_gains);
    void set_acceleration_limits(const Vector3f& limits);
    
    // Get outputs
    Vector3f get_velocity_command() const { return velocity_command; }
    Vector3f get_acceleration_command() const { return acceleration_command; }
    float get_thrust_command() const { return thrust_command; }
    
private:
    // System parameters
    HexacopterSystemID sys_id;
    
    // Control gains
    Vector3f pos_p_gains;
    Vector3f vel_p_gains;
    Vector3f vel_i_gains;
    Vector3f vel_d_gains;
    
    // Limits
    Vector3f accel_limits;
    Vector3f velocity_limits;
    
    // Controller states
    Vector3f velocity_integrator;
    Vector3f last_velocity_error;
    Vector3f velocity_command;
    Vector3f acceleration_command;
    float thrust_command;
    
    // Trajectory parameters
    struct TrajectoryCoeffs {
        Vector3f a0, a1, a2, a3, a4, a5;  // Polynomial coefficients
    };
    TrajectoryCoeffs trajectory_coeffs;
    
    // Helper functions
    void compute_trajectory_coeffs(const Vector3f& start_pos, const Vector3f& end_pos, float T);
    Vector3f saturate_vector(const Vector3f& input, const Vector3f& limits);
};

// EKF integration and state estimation interface
class HexacopterStateEstimator {
public:
    HexacopterStateEstimator();
    
    // Initialize EKF with specific parameters for hexacopter
    bool initialize(const HexacopterSystemID& system_id);
    
    // Update EKF with sensor data
    void update_imu(const Vector3f& gyro, const Vector3f& accel, float dt);
    void update_gps(const Vector3f& position, const Vector3f& velocity, float accuracy);
    void update_magnetometer(const Vector3f& mag_field);
    void update_barometer(float altitude, float accuracy);
    
    // Get state estimates
    Vector3f get_position() const;
    Vector3f get_velocity() const;
    Vector3f get_attitude() const;  // Euler angles
    Vector3f get_angular_rate() const;
    Quaternion get_quaternion() const;
    Vector3f get_gyro_bias() const;
    Vector3f get_accel_bias() const;
    
    // Get covariances and health
    Matrix3f get_position_covariance() const;
    Matrix3f get_velocity_covariance() const;
    Matrix3f get_attitude_covariance() const;
    bool is_healthy() const;
    float get_position_accuracy() const;
    
    // EKF status and diagnostics
    struct EKFStatus {
        bool position_valid;
        bool velocity_valid;
        bool attitude_valid;
        float innovation_magnitude[3];  // position innovations
        float innovation_test_ratio[3]; // innovation test ratios
        uint32_t last_update_time_ms;
    };
    
    EKFStatus get_status() const;
    
    // Reset functions
    void reset_position(const Vector3f& position);
    void reset_velocity(const Vector3f& velocity);
    void reset_attitude(const Vector3f& attitude);
    
private:
    // EKF instance
    NavEKF3* ekf;
    
    // EKF health monitoring
    uint32_t last_healthy_time_ms;
    float health_timeout_ms;
    
    // Innovation monitoring
    Vector3f last_innovations;
    Vector3f innovation_test_ratios;
    
    // Helper functions
    void check_ekf_health();
    void monitor_innovations();
    bool validate_measurement(const Vector3f& measurement, const Vector3f& limits);
};

// Main hexacopter control system integrating all components
class HexacopterControlSystem {
public:
    HexacopterControlSystem();
    
    // Initialization
    bool initialize();
    bool configure_for_hexacopter();
    
    // Main control loop (called at 400Hz)
    void update_control_loop();
    
    // Command interfaces
    void set_position_target(const Vector3f& position, float yaw);
    void set_velocity_target(const Vector3f& velocity, float yaw_rate);
    void set_attitude_target(const Vector3f& attitude, float thrust);
    void set_rate_target(const Vector3f& angular_rate, float thrust);
    
    // Flight mode interfaces
    void stabilize_mode(const Vector3f& pilot_input, float pilot_thrust);
    void altitude_hold_mode(const Vector3f& pilot_input, float target_altitude);
    void position_hold_mode(const Vector3f& position_target, float yaw_target);
    void guided_mode(const Vector3f& position_target, float yaw_target);
    
    // Safety and monitoring
    bool is_system_healthy() const;
    void emergency_stop();
    void set_safety_limits(float max_tilt_angle, float max_climb_rate, float max_horizontal_speed);
    
    // Tuning and parameters
    void set_attitude_gains(const Vector3f& rate_p, const Vector3f& rate_i, const Vector3f& rate_d,
                          const Vector3f& att_p);
    void set_position_gains(const Vector3f& pos_p, const Vector3f& vel_p, const Vector3f& vel_i);
    void update_system_identification(const HexacopterSystemID& new_sys_id);
    
    // Diagnostics and logging
    struct ControlDiagnostics {
        Vector3f attitude_error;
        Vector3f rate_error;
        Vector3f position_error;
        Vector3f velocity_error;
        float motor_outputs[6];
        float cpu_load_percent;
        uint32_t loop_time_us;
    };
    
    ControlDiagnostics get_diagnostics() const;
    void log_control_data();
    
private:
    // Control system components
    HexacopterSystemID system_id;
    HexacopterMixer* mixer;
    HexacopterAttitudeController* attitude_controller;
    HexacopterVelocityController* velocity_controller;
    HexacopterStateEstimator* state_estimator;
    
    // Control targets and states
    Vector3f position_target;
    Vector3f velocity_target;
    Vector3f attitude_target;
    Vector3f rate_target;
    float thrust_target;
    float yaw_target;
    
    // Safety limits
    float max_tilt_angle;
    float max_climb_rate;
    float max_horizontal_speed;
    
    // Timing and scheduling
    uint32_t last_update_time_us;
    uint32_t position_control_counter;
    uint32_t velocity_control_counter;
    
    // Diagnostics
    ControlDiagnostics diagnostics;
    uint32_t emergency_stop_time_ms;
    bool system_healthy;
    
    // Internal update functions
    void update_state_estimation();
    void update_position_control();
    void update_velocity_control();
    void update_attitude_control();
    void update_motor_outputs();
    void update_safety_checks();
    void update_diagnostics();
    
    // Helper functions
    bool check_control_limits(const Vector3f& attitude_cmd, float thrust_cmd);
    void apply_safety_limits(Vector3f& attitude_cmd, Vector3f& velocity_cmd);
    float calculate_cpu_load();
};