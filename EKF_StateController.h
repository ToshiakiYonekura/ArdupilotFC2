/*
 * EKF State Feedback Controller Implementation
 * 
 * This file implements an EKF-based state feedback controller for hexacopter
 * that uses the full EKF state vector and covariance matrix for optimal control.
 * Includes switching capability between traditional cascade and EKF feedback control.
 * 
 * Based on Linear Quadratic Gaussian (LQG) control theory.
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include "HexacopterController.h"

// Control mode selection
enum class ControlMode {
    CASCADE_CONTROL = 0,    // Traditional position→velocity→attitude cascade
    EKF_STATE_FEEDBACK = 1, // Direct EKF state feedback control
    ADAPTIVE_HYBRID = 2     // Hybrid approach with adaptive switching
};

// EKF state vector indices (for NavEKF3)
namespace EKFStateIndices {
    // Quaternion attitude (4 elements)
    static constexpr uint8_t QUAT_START = 0;
    static constexpr uint8_t QW = 0;
    static constexpr uint8_t QX = 1;
    static constexpr uint8_t QY = 2;
    static constexpr uint8_t QZ = 3;
    
    // Velocity NED (3 elements)
    static constexpr uint8_t VEL_START = 4;
    static constexpr uint8_t VN = 4;
    static constexpr uint8_t VE = 5;
    static constexpr uint8_t VD = 6;
    
    // Position NED (3 elements)
    static constexpr uint8_t POS_START = 7;
    static constexpr uint8_t PN = 7;
    static constexpr uint8_t PE = 8;
    static constexpr uint8_t PD = 9;
    
    // Gyro bias (3 elements)
    static constexpr uint8_t GYRO_BIAS_START = 10;
    static constexpr uint8_t GYRO_BIAS_X = 10;
    static constexpr uint8_t GYRO_BIAS_Y = 11;
    static constexpr uint8_t GYRO_BIAS_Z = 12;
    
    // Accelerometer bias (3 elements)  
    static constexpr uint8_t ACCEL_BIAS_START = 13;
    static constexpr uint8_t ACCEL_BIAS_X = 13;
    static constexpr uint8_t ACCEL_BIAS_Y = 14;
    static constexpr uint8_t ACCEL_BIAS_Z = 15;
    
    // Wind velocity (3 elements)
    static constexpr uint8_t WIND_START = 16;
    static constexpr uint8_t WIND_N = 16;
    static constexpr uint8_t WIND_E = 17;
    static constexpr uint8_t WIND_D = 18;
    
    // Earth magnetic field (3 elements)
    static constexpr uint8_t MAG_EARTH_START = 19;
    static constexpr uint8_t MAG_EARTH_X = 19;
    static constexpr uint8_t MAG_EARTH_Y = 20;
    static constexpr uint8_t MAG_EARTH_Z = 21;
    
    // Body magnetic field (3 elements)
    static constexpr uint8_t MAG_BODY_START = 22;
    static constexpr uint8_t MAG_BODY_X = 22;
    static constexpr uint8_t MAG_BODY_Y = 23;
    static constexpr uint8_t MAG_BODY_Z = 24;
    
    static constexpr uint8_t STATE_SIZE = 24;
};

// LQG controller configuration
struct LQGConfiguration {
    // State weighting matrix Q diagonal elements
    Vector3f position_weights;      // Position error weights [x, y, z]
    Vector3f velocity_weights;      // Velocity error weights [vx, vy, vz]  
    Vector3f attitude_weights;      // Attitude error weights [roll, pitch, yaw]
    Vector3f angular_rate_weights;  // Angular rate weights [p, q, r]
    
    // Control weighting matrix R diagonal elements
    Vector4f control_weights;       // Control effort weights [roll_torque, pitch_torque, yaw_torque, thrust]
    
    // Adaptation parameters
    float uncertainty_threshold;    // Threshold for switching to conservative control
    float adaptation_rate;          // Rate of adaptation to uncertainty
    float min_gain_factor;          // Minimum gain scaling factor
    float max_gain_factor;          // Maximum gain scaling factor
    
    // Default constructor with reasonable values
    LQGConfiguration() :
        position_weights(10.0f, 10.0f, 15.0f),
        velocity_weights(5.0f, 5.0f, 8.0f),
        attitude_weights(100.0f, 100.0f, 50.0f),
        angular_rate_weights(20.0f, 20.0f, 10.0f),
        control_weights(1.0f, 1.0f, 2.0f, 0.1f),
        uncertainty_threshold(0.5f),
        adaptation_rate(0.1f),
        min_gain_factor(0.3f),
        max_gain_factor(2.0f)
    {}
};

// EKF State Feedback Controller Class
class EKFStateController {
public:
    EKFStateController(const HexacopterSystemID& system_id);
    
    // Initialization and configuration
    bool initialize();
    void set_control_mode(ControlMode mode);
    void set_lqg_configuration(const LQGConfiguration& config);
    
    // Main control update (called at 400Hz)
    Vector4f update_control(const Vector3f& position_cmd, 
                           const Vector3f& velocity_cmd,
                           const Vector3f& attitude_cmd,
                           float yaw_rate_cmd,
                           float dt);
    
    // Control mode switching
    void enable_ekf_feedback_control();
    void enable_cascade_control();
    void enable_adaptive_hybrid_control();
    
    // Reference tracking
    void set_position_reference(const Vector3f& pos_ref, const Vector3f& vel_ref = Vector3f());
    void set_trajectory_reference(const Vector3f& pos_ref, const Vector3f& vel_ref, const Vector3f& acc_ref);
    
    // State and diagnostics
    Vector3f get_position_estimate() const;
    Vector3f get_velocity_estimate() const;
    Vector3f get_attitude_estimate() const;
    Matrix3f get_position_covariance() const;
    Matrix3f get_velocity_covariance() const;
    Matrix3f get_attitude_covariance() const;
    
    // Performance monitoring
    struct ControlDiagnostics {
        ControlMode active_mode;
        float estimation_uncertainty;
        float control_effort_magnitude;
        Vector3f position_error;
        Vector3f velocity_error;
        Vector3f attitude_error;
        Vector4f innovation_sequence;
        float adaptive_gain_factor;
        uint32_t mode_switch_count;
        bool ekf_healthy;
    };
    
    ControlDiagnostics get_diagnostics() const;
    
    // Advanced features
    void set_disturbance_rejection_gains(const Vector3f& gains);
    void enable_wind_compensation(bool enable);
    void set_reference_model_parameters(float natural_freq, float damping_ratio);
    
private:
    // System parameters
    HexacopterSystemID system_id;
    
    // EKF interface
    NavEKF3* ekf3;
    AP_AHRS* ahrs;
    
    // Control configuration
    ControlMode current_mode;
    LQGConfiguration lqg_config;
    
    // Traditional cascade controller (fallback)
    HexacopterControlSystem* cascade_controller;
    
    // LQG controller matrices
    Matrix<float, 12, 12> Q_matrix;     // State cost matrix (reduced state)
    Matrix<float, 4, 4> R_matrix;       // Control cost matrix
    Matrix<float, 4, 12> K_lqr;         // LQR feedback gain matrix
    Matrix<float, 4, 12> K_adaptive;    // Adaptive feedback gain matrix
    
    // State and reference vectors
    Vector<float, 12> state_estimate;   // Reduced state vector [pos, vel, att, rate]
    Vector<float, 12> state_reference;  // Reference state vector
    Vector<float, 12> state_error;      // State tracking error
    
    // Control outputs
    Vector4f control_output;            // [roll_torque, pitch_torque, yaw_torque, thrust]
    Vector4f control_limits;            // Maximum control efforts
    
    // Adaptation and switching logic
    float current_uncertainty;
    float adaptive_gain_factor;
    uint32_t mode_switch_count;
    uint32_t last_mode_switch_time_ms;
    float mode_switch_hysteresis;
    
    // Performance monitoring
    ControlDiagnostics diagnostics;
    
    // Reference model for trajectory tracking
    float reference_natural_freq;
    float reference_damping_ratio;
    
    // Wind compensation
    bool wind_compensation_enabled;
    Vector3f wind_estimate;
    Vector3f disturbance_rejection_gains;
    
    // Core EKF state feedback methods
    bool extract_ekf_state_vector();
    bool extract_ekf_covariance_matrix();
    Vector4f compute_lqg_control();
    Vector4f compute_adaptive_control();
    Vector4f compute_robust_control();
    
    // State processing
    void convert_ekf_to_control_state();
    void compute_state_error();
    Vector3f quaternion_to_euler(const Vector4f& quat) const;
    Vector3f compute_attitude_error(const Vector3f& att_ref, const Vector4f& quat_est) const;
    
    // Adaptation logic
    void update_adaptive_gains();
    void compute_estimation_uncertainty();
    bool should_switch_to_cascade();
    bool should_switch_to_ekf_feedback();
    
    // Control law implementations
    Vector4f lqr_control_law();
    Vector4f robust_h_infinity_control();
    Vector4f model_predictive_control();
    
    // Reference generation
    void update_reference_model(float dt);
    void generate_smooth_reference(const Vector3f& target_pos, const Vector3f& target_vel, float dt);
    
    // Safety and limits
    void apply_control_limits();
    void apply_rate_limits(float dt);
    bool check_control_authority();
    
    // Utility functions
    Matrix<float, 12, 12> compute_lqr_cost_matrix();
    Matrix<float, 4, 4> compute_control_cost_matrix();
    void update_diagnostics();
    float compute_control_effort_magnitude() const;
    
    // Innovation-based adaptation
    void monitor_innovation_sequence();
    void adapt_to_model_uncertainty();
    
    // Wind estimation and compensation
    void update_wind_estimate();
    Vector3f compute_wind_compensation();
    
    // Fault detection and isolation
    bool detect_actuator_faults();
    bool detect_sensor_faults();
    void reconfigure_for_faults();
};

// Control mode switching interface
class ControlModeManager {
public:
    ControlModeManager(EKFStateController* ekf_controller, HexacopterControlSystem* cascade_controller);
    
    // Automatic mode switching based on conditions
    void update_mode_selection(float dt);
    
    // Manual mode switching
    void request_mode_switch(ControlMode requested_mode);
    
    // Switching criteria configuration
    void set_switching_criteria(float uncertainty_threshold, 
                               float performance_threshold,
                               float stability_margin);
    
    // Performance evaluation
    struct ModePerformance {
        float tracking_error_rms;
        float control_effort_rms;
        float settling_time;
        float overshoot_percentage;
        bool stability_margin_ok;
    };
    
    ModePerformance evaluate_cascade_performance();
    ModePerformance evaluate_ekf_performance();
    
    // Get current status
    ControlMode get_active_mode() const;
    float get_performance_metric() const;
    bool is_mode_switching_enabled() const;
    
private:
    EKFStateController* ekf_ctrl;
    HexacopterControlSystem* cascade_ctrl;
    
    ControlMode active_mode;
    ControlMode requested_mode;
    
    // Switching criteria
    float uncertainty_threshold;
    float performance_threshold;
    float stability_margin_threshold;
    
    // Performance tracking
    float cascade_performance_score;
    float ekf_performance_score;
    
    // Switching state machine
    enum class SwitchingState {
        STABLE,
        EVALUATING,
        SWITCHING,
        MONITORING
    };
    
    SwitchingState switching_state;
    uint32_t evaluation_start_time;
    uint32_t switching_start_time;
    
    // Safety checks
    bool safe_to_switch_mode();
    void perform_switching_sequence();
    void monitor_post_switch_performance();
};

// Factory function for creating controllers
EKFStateController* create_ekf_state_controller(const HexacopterSystemID& system_id);
ControlModeManager* create_mode_manager(EKFStateController* ekf_ctrl, HexacopterControlSystem* cascade_ctrl);