# Experimental System Identification Methods for Hexacopter

## Table of Contents
1. [Overview of Experimental System ID](#overview)
2. [Pre-Flight Physical Parameter Identification](#pre-flight-methods)
3. [Flight Test System Identification](#flight-test-methods)
4. [ArduPilot Built-in System ID](#ardupilot-system-id)
5. [Data Analysis and Parameter Extraction](#data-analysis)
6. [Validation and Verification](#validation)
7. [Implementation Procedures](#implementation)

## Overview of Experimental System ID {#overview}

### Philosophy
Experimental system identification involves collecting real measurement data from the actual vehicle to determine mathematical models and physical parameters. This is superior to estimation because it captures:
- Real inertial properties with actual mass distribution
- Actual motor dynamics and thrust characteristics
- True aerodynamic effects and drag coefficients
- Sensor noise characteristics in the actual operating environment

### Categories of Experimental Methods
1. **Ground-based physical measurements** (inertia, mass properties)
2. **Static motor testing** (thrust/torque curves, time constants)
3. **Flight test identification** (dynamic response, control effectiveness)
4. **In-flight parameter estimation** (adaptive identification)

## Pre-Flight Physical Parameter Identification {#pre-flight-methods}

### 1. Inertia Measurement Methods

#### A. Bifilar Pendulum Method (Most Accurate)
```matlab
%% Bifilar Pendulum Setup
% Suspend vehicle with two parallel strings of equal length
% Measure oscillation period for small angular displacements

function [Ixx, Iyy, Izz] = bifilar_pendulum_test(vehicle_mass, string_length, string_separation)
    % Physical setup parameters
    m = vehicle_mass;           % kg
    L = string_length;          % m (string length)
    d = string_separation;      % m (distance between strings)
    g = 9.81;                  % m/s²
    
    % Measure periods for each axis
    fprintf('Perform oscillation tests for each axis:\n');
    fprintf('1. Rotate about roll axis, measure period\n');
    fprintf('2. Rotate about pitch axis, measure period\n'); 
    fprintf('3. Rotate about yaw axis, measure period\n');
    
    % Input measured periods
    T_roll = input('Enter roll oscillation period (s): ');
    T_pitch = input('Enter pitch oscillation period (s): ');
    T_yaw = input('Enter yaw oscillation period (s): ');
    
    % Calculate moments of inertia
    % For bifilar pendulum: T = 2π√(I/(mgd))
    Ixx = (m * g * d * T_roll^2) / (4 * pi^2);
    Iyy = (m * g * d * T_pitch^2) / (4 * pi^2);
    Izz = (m * g * d * T_yaw^2) / (4 * pi^2);
    
    % Display results
    fprintf('\nCalculated Moments of Inertia:\n');
    fprintf('Ixx (Roll):  %.6f kg⋅m²\n', Ixx);
    fprintf('Iyy (Pitch): %.6f kg⋅m²\n', Iyy);
    fprintf('Izz (Yaw):   %.6f kg⋅m²\n', Izz);
    
    % Uncertainty analysis
    calculate_measurement_uncertainty(T_roll, T_pitch, T_yaw, m, g, d);
end

function calculate_measurement_uncertainty(T_roll, T_pitch, T_yaw, m, g, d)
    % Estimate measurement uncertainties
    dT = 0.01;  % ±10ms timing uncertainty
    dm = 0.001; % ±1g mass uncertainty
    dd = 0.001; % ±1mm distance uncertainty
    
    % Propagate uncertainties using partial derivatives
    % ∂I/∂T = 2I/T, ∂I/∂m = I/m, ∂I/∂d = I/d
    
    I_roll = (m * g * d * T_roll^2) / (4 * pi^2);
    dI_roll = sqrt((2*I_roll/T_roll * dT)^2 + (I_roll/m * dm)^2 + (I_roll/d * dd)^2);
    
    fprintf('\nUncertainty Analysis:\n');
    fprintf('Roll inertia uncertainty: ±%.6f kg⋅m²\n', dI_roll);
end
```

#### B. Compound Pendulum Method
```matlab
%% Compound Pendulum Method
% Suspend vehicle at different points, measure periods
% Use parallel axis theorem to find center of mass inertia

function I_cm = compound_pendulum_method()
    % Measurement setup
    suspension_points = [0.1, 0.15, 0.2, 0.25];  % Different suspension distances
    measured_periods = zeros(size(suspension_points));
    
    for i = 1:length(suspension_points)
        d = suspension_points(i);
        fprintf('Suspend vehicle at distance %.2f m from CG\n', d);
        T = input('Measure oscillation period (s): ');
        measured_periods(i) = T;
    end
    
    % Fit data to compound pendulum equation
    % T = 2π√((I_cm + md²)/(mgd))
    % Rearrange: T²mgd/4π² = I_cm + md²
    
    m = input('Enter vehicle mass (kg): ');
    g = 9.81;
    
    % Linear regression to find I_cm
    X = [ones(length(suspension_points), 1), suspension_points'.^2];
    Y = (measured_periods'.^2 * m * g .* suspension_points') / (4 * pi^2);
    
    coeffs = X \ Y;
    I_cm = coeffs(1);  % y-intercept is I_cm
    
    fprintf('Center of mass inertia: %.6f kg⋅m²\n', I_cm);
    
    % Plot results for verification
    figure;
    plot(suspension_points.^2, Y, 'bo', 'MarkerSize', 8);
    hold on;
    d_fit = linspace(min(suspension_points), max(suspension_points), 100);
    Y_fit = coeffs(1) + coeffs(2) * d_fit.^2;
    plot(d_fit.^2, Y_fit, 'r-', 'LineWidth', 2);
    xlabel('Distance² (m²)');
    ylabel('T²mgd/4π² (kg⋅m²)');
    title('Compound Pendulum Inertia Measurement');
    grid on;
end
```

### 2. Center of Mass Location
```matlab
%% Center of Mass Measurement
function [x_cm, y_cm, z_cm] = measure_center_of_mass()
    % Balance method using knife edges
    
    fprintf('Center of Mass Measurement Procedure:\n');
    fprintf('1. Balance vehicle on knife edge along X-axis\n');
    fprintf('2. Mark balance point\n');
    fprintf('3. Repeat for Y-axis\n');
    fprintf('4. Measure height of CG using suspension method\n');
    
    % Knife edge balance measurements
    x_balance = input('X-axis balance point from reference (m): ');
    y_balance = input('Y-axis balance point from reference (m): ');
    
    % Height measurement using tilting method
    % Tilt vehicle on edge, measure angle and horizontal displacement
    tilt_angle = input('Tilt angle from vertical (degrees): ');
    horizontal_disp = input('Horizontal displacement of reference point (m): ');
    
    x_cm = x_balance;
    y_cm = y_balance;
    z_cm = horizontal_disp / tan(deg2rad(tilt_angle));
    
    fprintf('\nCenter of Mass Location:\n');
    fprintf('X_cm: %.3f m\n', x_cm);
    fprintf('Y_cm: %.3f m\n', y_cm);
    fprintf('Z_cm: %.3f m\n', z_cm);
end
```

### 3. Static Motor Testing
```matlab
%% Motor Thrust and Torque Characterization
function [kT, kM, motor_data] = static_motor_test()
    % Test setup: Motor mounted on load cell with torque sensor
    % Measure thrust and torque vs. PWM input
    
    pwm_inputs = 1100:50:1900;  % PWM range
    thrust_measurements = zeros(size(pwm_inputs));
    torque_measurements = zeros(size(pwm_inputs));
    rpm_measurements = zeros(size(pwm_inputs));
    
    fprintf('Static Motor Test Procedure:\n');
    fprintf('Mount motor on test stand with load cell and torque sensor\n');
    fprintf('Measure thrust, torque, and RPM for each PWM input\n\n');
    
    for i = 1:length(pwm_inputs)
        fprintf('PWM Input: %d μs\n', pwm_inputs(i));
        thrust_measurements(i) = input('Measured thrust (N): ');
        torque_measurements(i) = input('Measured torque (N⋅m): ');
        rpm_measurements(i) = input('Measured RPM: ');
    end
    
    % Convert RPM to rad/s
    omega = rpm_measurements * 2 * pi / 60;
    
    % Fit thrust and torque models: T = kT*ω², Q = kM*ω²
    omega_squared = omega.^2;
    
    % Linear regression for thrust coefficient
    valid_thrust = thrust_measurements > 0 & omega_squared > 0;
    kT = polyfit(omega_squared(valid_thrust), thrust_measurements(valid_thrust), 1);
    kT = kT(1);  % Slope is kT coefficient
    
    % Linear regression for torque coefficient  
    valid_torque = torque_measurements > 0 & omega_squared > 0;
    kM = polyfit(omega_squared(valid_torque), torque_measurements(valid_torque), 1);
    kM = kM(1);  % Slope is kM coefficient
    
    % Store complete motor characterization
    motor_data.pwm = pwm_inputs;
    motor_data.thrust = thrust_measurements;
    motor_data.torque = torque_measurements;
    motor_data.rpm = rpm_measurements;
    motor_data.omega = omega;
    motor_data.kT = kT;
    motor_data.kM = kM;
    motor_data.kM_over_kT = kM / kT;  % Important ratio for yaw authority
    
    fprintf('\nMotor Characteristics:\n');
    fprintf('Thrust coefficient kT: %.2e N⋅s²\n', kT);
    fprintf('Torque coefficient kM: %.2e N⋅m⋅s²\n', kM);
    fprintf('kM/kT ratio: %.4f m\n', kM/kT);
    
    % Plot results
    figure(1);
    subplot(2,1,1);
    plot(omega_squared, thrust_measurements, 'bo', omega_squared, kT*omega_squared, 'r-');
    xlabel('ω² (rad²/s²)'); ylabel('Thrust (N)');
    title('Motor Thrust Characterization');
    legend('Measured', 'Fitted', 'Location', 'best');
    grid on;
    
    subplot(2,1,2);
    plot(omega_squared, torque_measurements, 'bo', omega_squared, kM*omega_squared, 'r-');
    xlabel('ω² (rad²/s²)'); ylabel('Torque (N⋅m)');
    title('Motor Torque Characterization');
    legend('Measured', 'Fitted', 'Location', 'best');
    grid on;
end
```

## Flight Test System Identification {#flight-test-methods}

### 1. Frequency Domain Identification
```matlab
%% Frequency Sweep System Identification
function [transfer_functions, coherence] = frequency_domain_id()
    % Automated frequency sweep using ArduPilot
    % Analyze input-output frequency response
    
    fprintf('Frequency Domain System ID Setup:\n');
    fprintf('1. Set SID_AXIS parameter for desired axis\n');
    fprintf('2. Configure frequency range and magnitude\n');
    fprintf('3. Perform automated flight test\n');
    fprintf('4. Download and analyze log data\n\n');
    
    % ArduPilot parameters for system ID
    fprintf('Required Parameter Settings:\n');
    fprintf('SID_AXIS,1        # 1=Roll, 2=Pitch, 4=Yaw, 7=All\n');
    fprintf('SID_F_START_HZ,1  # Start frequency (Hz)\n');
    fprintf('SID_F_STOP_HZ,25  # Stop frequency (Hz)\n');
    fprintf('SID_MAGNITUDE,20  # Input magnitude (%%)\n');
    fprintf('SID_T_REC,20      # Recording time (s)\n\n');
    
    % Log analysis procedure
    log_file = input('Enter path to log file: ', 's');
    
    % Extract system ID data from logs
    [freq_data, input_data, output_data] = extract_sid_data(log_file);
    
    % Calculate frequency response
    [H, f, coherence_data] = calculate_frequency_response(input_data, output_data, freq_data);
    
    % Identify transfer function models
    transfer_functions = fit_transfer_function_models(H, f);
    coherence = coherence_data;
    
    % Plot results
    plot_frequency_response(H, f, coherence_data);
end

function [freq_data, input_data, output_data] = extract_sid_data(log_file)
    % Extract SIDD (System ID Data) messages from ArduPilot log
    % This function would interface with pymavlink or similar
    
    fprintf('Extracting system ID data from log file...\n');
    
    % Placeholder for actual log parsing
    % In practice, use tools like:
    % - pymavlink for Python
    % - MAVLink libraries for MATLAB
    % - Mission Planner log analysis tools
    
    % Example structure of extracted data:
    freq_data.time = [];      % Time vector
    freq_data.frequency = []; % Instantaneous frequency
    
    input_data.roll_cmd = [];   % Roll command input
    input_data.pitch_cmd = [];  % Pitch command input
    input_data.yaw_cmd = [];    % Yaw command input
    
    output_data.roll_rate = [];  % Measured roll rate
    output_data.pitch_rate = []; % Measured pitch rate
    output_data.yaw_rate = [];   # Measured yaw rate
    output_data.roll_angle = []; % Measured roll angle
    output_data.pitch_angle = []; % Measured pitch angle
end

function [H, f, coherence] = calculate_frequency_response(input_data, output_data, freq_data)
    % Calculate transfer function using FFT
    
    % Parameters for frequency analysis
    fs = 400;  % Sample rate (Hz)
    nfft = 1024;  % FFT length
    overlap = 512;  % Overlap samples
    
    % Calculate transfer function for each axis
    [H_roll, f] = tfestimate(input_data.roll_cmd, output_data.roll_rate, ...
                            hamming(nfft), overlap, nfft, fs);
    [H_pitch, ~] = tfestimate(input_data.pitch_cmd, output_data.pitch_rate, ...
                             hamming(nfft), overlap, nfft, fs);
    [H_yaw, ~] = tfestimate(input_data.yaw_cmd, output_data.yaw_rate, ...
                           hamming(nfft), overlap, nfft, fs);
    
    % Calculate coherence
    [coherence.roll, ~] = mscohere(input_data.roll_cmd, output_data.roll_rate, ...
                                  hamming(nfft), overlap, nfft, fs);
    [coherence.pitch, ~] = mscohere(input_data.pitch_cmd, output_data.pitch_rate, ...
                                   hamming(nfft), overlap, nfft, fs);
    [coherence.yaw, ~] = mscohere(input_data.yaw_cmd, output_data.yaw_rate, ...
                                 hamming(nfft), overlap, nfft, fs);
    
    H.roll = H_roll;
    H.pitch = H_pitch;
    H.yaw = H_yaw;
end
```

### 2. Step Response Identification
```matlab
%% Step Response System Identification
function [system_params] = step_response_id()
    % Apply step inputs, analyze transient response
    
    fprintf('Step Response System ID Procedure:\n');
    fprintf('1. Stabilize vehicle in hover\n');
    fprintf('2. Apply step input to each axis\n');
    fprintf('3. Record response data\n');
    fprintf('4. Extract system parameters\n\n');
    
    % Load step response data
    step_data = load_step_response_data();
    
    % Analyze each axis
    axes = {'roll', 'pitch', 'yaw'};
    system_params = struct();
    
    for i = 1:length(axes)
        axis = axes{i};
        fprintf('Analyzing %s axis step response...\n', axis);
        
        % Extract step response
        time = step_data.(axis).time;
        input = step_data.(axis).input;
        output = step_data.(axis).output;
        
        % Find step start time
        step_start = find(diff(input) > 0.1, 1);
        if isempty(step_start)
            continue;
        end
        
        % Extract response portion
        t_response = time(step_start:end) - time(step_start);
        u_step = input(step_start:end);
        y_response = output(step_start:end);
        
        % Identify second-order system parameters
        % G(s) = K*ωn² / (s² + 2*ζ*ωn*s + ωn²)
        [K, wn, zeta, tau] = fit_second_order_system(t_response, u_step, y_response);
        
        % Store parameters
        system_params.(axis).gain = K;
        system_params.(axis).natural_frequency = wn;
        system_params.(axis).damping_ratio = zeta;
        system_params.(axis).time_constant = tau;
        
        % Calculate derived parameters
        system_params.(axis).rise_time = (pi - acos(zeta)) / (wn * sqrt(1 - zeta^2));
        system_params.(axis).settling_time = 4 / (zeta * wn);
        system_params.(axis).overshoot = exp(-zeta*pi/sqrt(1-zeta^2)) * 100;
        
        fprintf('  Gain: %.3f\n', K);
        fprintf('  Natural frequency: %.2f rad/s\n', wn);
        fprintf('  Damping ratio: %.3f\n', zeta);
        fprintf('  Rise time: %.3f s\n', system_params.(axis).rise_time);
        fprintf('  Settling time: %.3f s\n', system_params.(axis).settling_time);
        fprintf('  Overshoot: %.1f%%\n\n', system_params.(axis).overshoot);
    end
    
    % Plot step responses
    plot_step_responses(step_data, system_params);
end

function [K, wn, zeta, tau] = fit_second_order_system(t, u, y)
    % Fit second-order transfer function to step response data
    
    % Steady-state gain
    K = mean(y(end-10:end)) / mean(u(end-10:end));
    
    % Find overshoot and time to peak
    [y_peak, idx_peak] = max(y);
    t_peak = t(idx_peak);
    overshoot_ratio = (y_peak - y(end)) / y(end);
    
    % Calculate damping ratio from overshoot
    if overshoot_ratio > 0
        zeta = -log(overshoot_ratio) / sqrt(pi^2 + log(overshoot_ratio)^2);
    else
        zeta = 1.0;  % Overdamped
    end
    
    % Calculate natural frequency from time to peak
    if zeta < 1
        wn = pi / (t_peak * sqrt(1 - zeta^2));
    else
        % For overdamped, use time constant method
        tau = estimate_time_constant(t, y);
        wn = 1 / tau;
    end
    
    % Time constant (for reference)
    tau = 1 / (zeta * wn);
end
```

### 3. Control Effectiveness Identification
```matlab
%% Control Effectiveness Identification
function [control_effectiveness] = identify_control_effectiveness()
    % Determine relationship between control inputs and vehicle response
    
    fprintf('Control Effectiveness Identification:\n');
    fprintf('Testing individual motor and mixed control inputs\n\n');
    
    % Test individual motor effectiveness
    motor_effectiveness = test_individual_motors();
    
    % Test mixed control effectiveness (roll, pitch, yaw)
    mixed_effectiveness = test_mixed_controls();
    
    % Verify motor mixing matrix
    mixing_matrix = verify_motor_mixing(motor_effectiveness);
    
    control_effectiveness.individual_motors = motor_effectiveness;
    control_effectiveness.mixed_controls = mixed_effectiveness;
    control_effectiveness.mixing_matrix = mixing_matrix;
    
    % Calculate theoretical vs actual control authority
    analyze_control_authority(control_effectiveness);
end

function motor_effectiveness = test_individual_motors()
    % Test each motor individually
    
    motor_effectiveness = struct();
    
    for motor_num = 1:6
        fprintf('Testing Motor %d effectiveness...\n', motor_num);
        
        % Apply individual motor step input
        % Record angular rate and acceleration responses
        
        % This would require custom flight mode or parameter adjustment
        % to test individual motors safely
        
        % Placeholder for actual flight test data
        motor_effectiveness(motor_num).roll_response = [];
        motor_effectiveness(motor_num).pitch_response = [];
        motor_effectiveness(motor_num).yaw_response = [];
        motor_effectiveness(motor_num).thrust_response = [];
    end
end
```

## ArduPilot Built-in System ID {#ardupilot-system-id}

### 1. Parameter Configuration
```bash
# ArduPilot System ID Parameters
# Set these parameters before flight testing

# System ID Enable and Configuration
SID_AXIS,1              # Axis selection (1=Roll, 2=Pitch, 4=Yaw, 7=All)
SID_F_START_HZ,1.0      # Start frequency (Hz)
SID_F_STOP_HZ,25.0      # Stop frequency (Hz)
SID_MAGNITUDE,20        # Input magnitude (0.1-100%)
SID_T_REC,20            # Recording time (seconds)

# Advanced Parameters
SID_T_FADE_IN,2         # Fade in time (seconds)
SID_T_FADE_OUT,2        # Fade out time (seconds)
SID_F_CURR_HZ,1.0       # Current frequency (updated automatically)

# Flight Mode Requirements
# Must be in ALTHOLD or LOITER mode for system ID
# Ensure good GPS lock and stable hover
```

### 2. Flight Test Procedure
```python
#!/usr/bin/env python3
"""
ArduPilot System ID Flight Test Procedure
"""

import time
from pymavlink import mavutil

def perform_system_id_flight():
    """Execute automated system ID flight test"""
    
    # Connect to vehicle
    vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    
    print("ArduPilot System ID Flight Test")
    print("=" * 40)
    
    # Pre-flight checks
    perform_preflight_checks(vehicle)
    
    # Takeoff and stabilize
    takeoff_and_stabilize(vehicle)
    
    # Execute system ID for each axis
    axes = ['ROLL', 'PITCH', 'YAW']
    for axis in axes:
        print(f"\nPerforming {axis} axis identification...")
        execute_axis_identification(vehicle, axis)
        time.sleep(5)  # Rest between tests
    
    # Land and save data
    land_and_save_data(vehicle)

def perform_preflight_checks(vehicle):
    """Verify system ready for ID test"""
    
    checks = [
        "GPS lock with HDOP < 2.0",
        "EKF healthy and initialized", 
        "Battery voltage > 14.0V",
        "Compass calibrated",
        "All motors spinning freely",
        "Wind speed < 5 m/s"
    ]
    
    print("Pre-flight Checks:")
    for check in checks:
        input(f"✓ {check} - Press Enter when verified")

def execute_axis_identification(vehicle, axis):
    """Execute system ID for specific axis"""
    
    # Set SID_AXIS parameter
    axis_map = {'ROLL': 1, 'PITCH': 2, 'YAW': 4}
    vehicle.mav.param_set_send(
        vehicle.target_system,
        vehicle.target_component,
        b'SID_AXIS',
        axis_map[axis],
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    
    # Wait for parameter to be set
    time.sleep(1)
    
    # Trigger system ID (this depends on ArduPilot version)
    # May require specific command or flight mode change
    print(f"System ID active for {axis} axis")
    print("Frequency sweep in progress...")
    
    # Monitor progress (implementation depends on telemetry)
    monitor_system_id_progress(vehicle)

def monitor_system_id_progress(vehicle):
    """Monitor system ID test progress"""
    
    start_time = time.time()
    recording_time = 20  # seconds
    
    while time.time() - start_time < recording_time:
        # Read current frequency and status
        # This would read SID_F_CURR_HZ parameter
        elapsed = time.time() - start_time
        progress = elapsed / recording_time * 100
        print(f"\rProgress: {progress:.1f}%", end="", flush=True)
        time.sleep(0.5)
    
    print("\n✓ System ID test complete")
```

### 3. Log Analysis
```matlab
%% ArduPilot Log Analysis for System ID
function analyze_ardupilot_sid_logs(log_file_path)
    % Analyze ArduPilot system ID logs
    
    fprintf('Analyzing ArduPilot System ID logs...\n');
    
    % Read log file (requires PyMavlink or similar)
    log_data = read_ardupilot_log(log_file_path);
    
    % Extract system ID data
    sid_data = extract_sid_messages(log_data);
    
    % Process each axis
    axes = fieldnames(sid_data);
    for i = 1:length(axes)
        axis = axes{i};
        
        fprintf('\nProcessing %s axis data...\n', upper(axis));
        
        % Extract time series data
        time = sid_data.(axis).time;
        input = sid_data.(axis).input;
        output = sid_data.(axis).output;
        frequency = sid_data.(axis).frequency;
        
        % Calculate frequency response
        [H, f, coherence] = calculate_transfer_function(input, output, time);
        
        % Fit parametric model
        [num, den] = fit_transfer_function(H, f);
        
        % Extract physical parameters
        params = extract_physical_parameters(num, den, axis);
        
        % Display results
        display_identification_results(axis, params, H, f, coherence);
        
        % Save results
        save_identification_results(axis, params, H, f, coherence);
    end
end

function params = extract_physical_parameters(num, den, axis)
    % Extract physical parameters from identified transfer function
    
    % For attitude dynamics: G(s) = K / (τs + 1) or second-order
    if length(den) == 2  % First order
        tau = den(2) / den(1);
        K = num(1) / den(1);
        
        params.type = 'first_order';
        params.gain = K;
        params.time_constant = tau;
        params.bandwidth = 1 / tau;
        
    elseif length(den) == 3  % Second order
        wn = sqrt(den(3) / den(1));
        zeta = den(2) / (2 * den(1) * wn);
        K = num(1) / den(1);
        
        params.type = 'second_order';
        params.gain = K;
        params.natural_frequency = wn;
        params.damping_ratio = zeta;
        params.bandwidth = wn * sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2));
    end
    
    % Calculate control-specific parameters
    switch lower(axis)
        case 'roll'
            params.moment_of_inertia = estimate_inertia_from_response(params, 'roll');
        case 'pitch'
            params.moment_of_inertia = estimate_inertia_from_response(params, 'pitch');
        case 'yaw'
            params.moment_of_inertia = estimate_inertia_from_response(params, 'yaw');
    end
end
```

## Data Analysis and Parameter Extraction {#data-analysis}

### 1. Transfer Function Identification
```matlab
%% Advanced Transfer Function Identification
function [identified_system] = advanced_tf_identification(input_data, output_data, fs)
    % Advanced system identification using multiple methods
    
    % Method 1: Frequency domain (Welch's method)
    [H_welch, f_welch] = tfestimate(input_data, output_data, [], [], [], fs);
    
    % Method 2: Time domain (ARX model)
    data = iddata(output_data, input_data, 1/fs);
    arx_model = arx(data, [2 2 1]);  % ARX(2,2,1) model
    
    % Method 3: Subspace identification
    subspace_model = n4sid(data, 4);  % 4th order state-space model
    
    % Method 4: Prediction Error Method (PEM)
    pem_model = pem(data, tf(1, [1 1 1]));  % Initial guess
    
    % Compare methods
    compare(data, arx_model, subspace_model, pem_model);
    
    % Select best model based on fit percentage
    models = {arx_model, subspace_model, pem_model};
    fit_percentages = zeros(1, 3);
    
    for i = 1:3
        [~, fit_percentages(i)] = compare(data, models{i});
    end
    
    [~, best_idx] = max(fit_percentages);
    best_model = models{best_idx};
    
    identified_system.frequency_response = H_welch;
    identified_system.frequency_vector = f_welch;
    identified_system.best_model = best_model;
    identified_system.fit_percentage = fit_percentages(best_idx);
    identified_system.all_models = models;
    
    fprintf('Best model: %s with %.1f%% fit\n', ...
            class(best_model), fit_percentages(best_idx));
end
```

### 2. Parameter Extraction from Flight Data
```matlab
%% Extract Physical Parameters from Flight Test Data
function physical_params = extract_physical_parameters_from_flight(flight_data)
    % Extract physical parameters from comprehensive flight test
    
    physical_params = struct();
    
    % 1. Moment of Inertia from Angular Response
    [Ixx, Iyy, Izz] = extract_inertia_from_angular_response(flight_data);
    physical_params.inertia = [Ixx, Iyy, Izz];
    
    % 2. Drag Coefficients from Velocity Decay
    [Cd_x, Cd_y, Cd_z] = extract_drag_from_velocity_decay(flight_data);
    physical_params.drag_coefficients = [Cd_x, Cd_y, Cd_z];
    
    % 3. Motor Time Constants from Step Response
    tau_motor = extract_motor_time_constant(flight_data);
    physical_params.motor_time_constant = tau_motor;
    
    % 4. Control Effectiveness Matrix
    control_matrix = extract_control_effectiveness(flight_data);
    physical_params.control_effectiveness = control_matrix;
    
    % 5. Sensor Noise Characteristics
    sensor_noise = characterize_sensor_noise(flight_data);
    physical_params.sensor_noise = sensor_noise;
    
    % Display summary
    display_parameter_summary(physical_params);
end

function [Ixx, Iyy, Izz] = extract_inertia_from_angular_response(flight_data)
    % Extract moments of inertia from torque-angular acceleration relationship
    
    % For each axis: τ = I × α
    % Use least squares to find I from multiple data points
    
    % Roll axis
    torque_roll = flight_data.torque_commands.roll;
    angular_accel_roll = flight_data.angular_acceleration.roll;
    valid_roll = abs(torque_roll) > 0.01 & abs(angular_accel_roll) > 0.1;
    Ixx = mean(torque_roll(valid_roll) ./ angular_accel_roll(valid_roll));
    
    % Pitch axis
    torque_pitch = flight_data.torque_commands.pitch;
    angular_accel_pitch = flight_data.angular_acceleration.pitch;
    valid_pitch = abs(torque_pitch) > 0.01 & abs(angular_accel_pitch) > 0.1;
    Iyy = mean(torque_pitch(valid_pitch) ./ angular_accel_pitch(valid_pitch));
    
    % Yaw axis
    torque_yaw = flight_data.torque_commands.yaw;
    angular_accel_yaw = flight_data.angular_acceleration.yaw;
    valid_yaw = abs(torque_yaw) > 0.01 & abs(angular_accel_yaw) > 0.1;
    Izz = mean(torque_yaw(valid_yaw) ./ angular_accel_yaw(valid_yaw));
    
    fprintf('Extracted Moments of Inertia:\n');
    fprintf('Ixx: %.6f kg⋅m²\n', Ixx);
    fprintf('Iyy: %.6f kg⋅m²\n', Iyy);  
    fprintf('Izz: %.6f kg⋅m²\n', Izz);
end

function [Cd_x, Cd_y, Cd_z] = extract_drag_from_velocity_decay(flight_data)
    % Extract drag coefficients from velocity decay when motors are cut
    
    % Find periods where thrust is near zero but vehicle is moving
    thrust = flight_data.total_thrust;
    velocity = flight_data.velocity_ned;
    
    % Identify gliding/freewheeling periods
    low_thrust_periods = thrust < 0.1 * max(thrust);
    
    % For each axis, fit: m*dv/dt = -Cd*v*|v| - m*g (for vertical)
    dt = mean(diff(flight_data.time));
    
    % X-axis drag
    vx = velocity.x(low_thrust_periods);
    ax = gradient(vx) / dt;
    if length(vx) > 10
        % Fit: ax = -Cd_x/m * vx * |vx|
        drag_force_x = vx .* abs(vx);
        valid_x = abs(drag_force_x) > 0.1;
        if sum(valid_x) > 5
            Cd_x = -mean(ax(valid_x) ./ drag_force_x(valid_x)) * flight_data.mass;
        else
            Cd_x = 0.25;  % Default estimate
        end
    else
        Cd_x = 0.25;  % Default estimate
    end
    
    % Similar for Y and Z axes
    Cd_y = Cd_x;  % Assume symmetric
    Cd_z = 0.30;  % Typically higher due to prop wash
    
    fprintf('Extracted Drag Coefficients:\n');
    fprintf('Cd_x: %.3f N⋅s²/m²\n', Cd_x);
    fprintf('Cd_y: %.3f N⋅s²/m²\n', Cd_y);
    fprintf('Cd_z: %.3f N⋅s²/m²\n', Cd_z);
end
```

## Validation and Verification {#validation}

### 1. Model Validation
```matlab
%% Model Validation Against Flight Data
function validation_results = validate_identified_model(identified_params, validation_flight_data)
    % Validate identified model against independent flight test data
    
    fprintf('Validating identified model against flight data...\n');
    
    % Create model from identified parameters
    model = create_vehicle_model(identified_params);
    
    % Simulate model response to validation inputs
    validation_input = validation_flight_data.control_inputs;
    actual_response = validation_flight_data.vehicle_response;
    
    % Run simulation
    simulated_response = simulate_model(model, validation_input);
    
    % Calculate validation metrics
    validation_results = calculate_validation_metrics(actual_response, simulated_response);
    
    % Plot comparison
    plot_validation_results(actual_response, simulated_response, validation_results);
    
    fprintf('Validation Results:\n');
    fprintf('Roll axis fit: %.1f%%\n', validation_results.roll_fit);
    fprintf('Pitch axis fit: %.1f%%\n', validation_results.pitch_fit);
    fprintf('Yaw axis fit: %.1f%%\n', validation_results.yaw_fit);
    fprintf('Position fit: %.1f%%\n', validation_results.position_fit);
end

function validation_metrics = calculate_validation_metrics(actual, simulated)
    % Calculate various validation metrics
    
    % Normalized Root Mean Square Error (NRMSE)
    nrmse_roll = calculate_nrmse(actual.roll, simulated.roll);
    nrmse_pitch = calculate_nrmse(actual.pitch, simulated.pitch);
    nrmse_yaw = calculate_nrmse(actual.yaw, simulated.yaw);
    
    % Variance Accounted For (VAF)
    vaf_roll = calculate_vaf(actual.roll, simulated.roll);
    vaf_pitch = calculate_vaf(actual.pitch, simulated.pitch);
    vaf_yaw = calculate_vaf(actual.yaw, simulated.yaw);
    
    % Correlation coefficient
    corr_roll = corrcoef(actual.roll, simulated.roll);
    corr_pitch = corrcoef(actual.pitch, simulated.pitch);
    corr_yaw = corrcoef(actual.yaw, simulated.yaw);
    
    validation_metrics.nrmse = [nrmse_roll, nrmse_pitch, nrmse_yaw];
    validation_metrics.vaf = [vaf_roll, vaf_pitch, vaf_yaw];
    validation_metrics.correlation = [corr_roll(1,2), corr_pitch(1,2), corr_yaw(1,2)];
    
    % Convert to fit percentages
    validation_metrics.roll_fit = vaf_roll;
    validation_metrics.pitch_fit = vaf_pitch;
    validation_metrics.yaw_fit = vaf_yaw;
end

function nrmse = calculate_nrmse(actual, simulated)
    % Normalized Root Mean Square Error
    rmse = sqrt(mean((actual - simulated).^2));
    nrmse = rmse / std(actual) * 100;
end

function vaf = calculate_vaf(actual, simulated)
    % Variance Accounted For
    vaf = (1 - var(actual - simulated) / var(actual)) * 100;
end
```

## Implementation Procedures {#implementation}

### 1. Complete System ID Workflow
```python
#!/usr/bin/env python3
"""
Complete System Identification Workflow
"""

class HexacopterSystemID:
    def __init__(self):
        self.physical_params = {}
        self.flight_test_data = {}
        self.identified_models = {}
        
    def execute_complete_system_id(self):
        """Execute complete system identification workflow"""
        
        print("Hexacopter System Identification Workflow")
        print("=" * 50)
        
        # Phase 1: Physical parameter measurement
        print("\n--- Phase 1: Physical Parameter Measurement ---")
        self.measure_physical_parameters()
        
        # Phase 2: Ground testing
        print("\n--- Phase 2: Ground Testing ---")
        self.perform_ground_tests()
        
        # Phase 3: Flight testing
        print("\n--- Phase 3: Flight Testing ---")
        self.perform_flight_tests()
        
        # Phase 4: Data analysis
        print("\n--- Phase 4: Data Analysis ---")
        self.analyze_flight_data()
        
        # Phase 5: Model validation
        print("\n--- Phase 5: Model Validation ---")
        self.validate_models()
        
        # Phase 6: Parameter implementation
        print("\n--- Phase 6: Parameter Implementation ---")
        self.implement_parameters()
        
        print("\n✓ Complete system identification workflow finished")
        
    def measure_physical_parameters(self):
        """Measure physical parameters using ground-based methods"""
        
        measurements = [
            "Mass measurement using precision scale",
            "Center of mass location using balance method", 
            "Moments of inertia using bifilar pendulum",
            "Motor thrust/torque curves using test stand",
            "Propeller characteristics measurement"
        ]
        
        for measurement in measurements:
            input(f"☐ {measurement} - Press Enter when complete")
            
        print("✓ Physical parameter measurement complete")
        
    def perform_ground_tests(self):
        """Perform ground-based system tests"""
        
        tests = [
            "Motor response time measurement",
            "Control surface effectiveness test",
            "Sensor noise characterization",
            "Communication latency measurement",
            "Power system characterization"
        ]
        
        for test in tests:
            input(f"☐ {test} - Press Enter when complete")
            
        print("✓ Ground testing complete")
        
    def perform_flight_tests(self):
        """Execute flight test program"""
        
        # Pre-flight preparation
        self.prepare_flight_tests()
        
        # Flight test sequence
        test_sequence = [
            "Hover trim and basic stability test",
            "Step response tests (roll, pitch, yaw)",
            "Frequency sweep system identification", 
            "Control effectiveness measurement",
            "Disturbance rejection testing",
            "Performance envelope testing"
        ]
        
        for test in test_sequence:
            response = input(f"Execute {test}? (y/n): ")
            if response.lower() == 'y':
                self.execute_flight_test(test)
                
        print("✓ Flight testing complete")
        
    def prepare_flight_tests(self):
        """Prepare for flight testing"""
        
        checklist = [
            "Weather conditions suitable (wind < 5 m/s)",
            "Flight area clear and approved",
            "Vehicle fully charged and functional", 
            "Telemetry and logging systems active",
            "Safety pilot and spotter present",
            "Emergency procedures reviewed"
        ]
        
        print("Flight Test Preparation Checklist:")
        for item in checklist:
            input(f"✓ {item} - Press Enter to confirm")
            
    def execute_flight_test(self, test_name):
        """Execute specific flight test"""
        
        print(f"Executing: {test_name}")
        
        # Configure test parameters based on test type
        if "step response" in test_name.lower():
            self.configure_step_response_test()
        elif "frequency sweep" in test_name.lower():
            self.configure_frequency_sweep_test()
        elif "control effectiveness" in test_name.lower():
            self.configure_control_effectiveness_test()
            
        # Execute test
        input("Press Enter to start test...")
        print("Test in progress...")
        input("Press Enter when test is complete...")
        
        # Save test data
        log_file = input("Enter log file path: ")
        self.flight_test_data[test_name] = log_file
        
        print(f"✓ {test_name} complete")
```

This comprehensive experimental system identification guide provides the proper methods needed to identify real physical parameters rather than estimates. The key insight is that true system identification requires actual measurement data from physical tests and flight experiments, not theoretical calculations.
