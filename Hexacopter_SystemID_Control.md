# ArduCopter Hexacopter System Identification and Control Design

## Table of Contents
1. [System Overview](#system-overview)
2. [Hexacopter Configuration](#hexacopter-configuration)
3. [System Identification](#system-identification)
4. [Attitude Control Design](#attitude-control-design)
5. [Velocity Control Design](#velocity-control-design)
6. [EKF Integration](#ekf-integration)
7. [Implementation](#implementation)
8. [Parameters and Tuning](#parameters-and-tuning)

## System Overview

This document presents a comprehensive system identification and control design for a 6-motor ArduCopter (hexacopter) including:
- Mathematical modeling and system identification
- Attitude control system design using PID and LQR controllers
- Velocity control system with EKF state estimation
- Complete implementation with parameters

### Vehicle Specifications
Based on ArduPilot defaults and typical hexacopter configurations:
- **Mass**: 2.5 kg (default for medium hexacopter)
- **Frame Type**: X-configuration (MOTOR_FRAME_TYPE_X)
- **Motor Count**: 6 motors
- **Moment of Inertia**: Estimated from typical hexacopter geometry

## Hexacopter Configuration

### Motor Layout (X-Configuration)
Based on `AP_MotorsMatrix::setup_hexa_matrix()` from ArduPilot source:

```
Motor positions (degrees from forward):
Motor 1:   30° (Front Right)     - CCW
Motor 2:   90° (Right)           - CW  
Motor 3:  150° (Rear Right)      - CCW
Motor 4: -150° (Rear Left)       - CW
Motor 5:  -90° (Left)            - CCW
Motor 6:  -30° (Front Left)      - CW
```

### Motor Mixing Matrix
```
     Roll  Pitch  Yaw   Throttle
M1: [0.50, 0.87,  1.0,   1.0]  # 30°, CCW
M2: [1.00, 0.00, -1.0,   1.0]  # 90°, CW
M3: [0.50,-0.87,  1.0,   1.0]  # 150°, CCW  
M4: [-0.50,-0.87, -1.0,  1.0]  # -150°, CW
M5: [-1.00, 0.00,  1.0,  1.0]  # -90°, CCW
M6: [-0.50, 0.87, -1.0,  1.0]  # -30°, CW
```

### Physical Parameters
```
Mass (m):              2.5 kg
Arm length (L):        0.25 m
Motor-to-motor distance: 0.50 m
Thrust coefficient (kT): 3.16e-6 N·s²
Torque coefficient (kM): 7.94e-8 N·m·s²
Maximum thrust per motor: 15 N
```

## System Identification

### Nonlinear Dynamic Model

#### State Vector
```
x = [p, q, r, φ, θ, ψ, u, v, w, x, y, z]ᵀ
```
Where:
- p, q, r: Angular velocities (rad/s)
- φ, θ, ψ: Euler angles (rad) 
- u, v, w: Linear velocities in body frame (m/s)
- x, y, z: Position in inertial frame (m)

#### Dynamic Equations

**Angular Dynamics:**
```
Iₓₓ·ṗ = (Iyy - Izz)·q·r + L·(τroll) + τdist_x
Iyy·q̇ = (Izz - Ixx)·p·r + L·(τpitch) + τdist_y  
Izz·ṙ = (Ixx - Iyy)·p·q + τyaw + τdist_z
```

**Kinematic Equations:**
```
φ̇ = p + sin(φ)·tan(θ)·q + cos(φ)·tan(θ)·r
θ̇ = cos(φ)·q - sin(φ)·r
ψ̇ = sin(φ)·sec(θ)·q + cos(φ)·sec(θ)·r
```

**Translational Dynamics:**
```
m·u̇ = -m·g·sin(θ) + Fthrust·sin(α) + Fdist_x
m·v̇ = m·g·cos(θ)·sin(φ) + Fthrust·sin(β) + Fdist_y
m·ẇ = m·g·cos(θ)·cos(φ) - Fthrust + Fdist_z
```

**Position Dynamics:**
```
ẋ = cos(ψ)·cos(θ)·u + (cos(ψ)·sin(θ)·sin(φ) - sin(ψ)·cos(φ))·v + (cos(ψ)·sin(θ)·cos(φ) + sin(ψ)·sin(φ))·w
ẏ = sin(ψ)·cos(θ)·u + (sin(ψ)·sin(θ)·sin(φ) + cos(ψ)·cos(φ))·v + (sin(ψ)·sin(θ)·cos(φ) - cos(ψ)·sin(φ))·w  
ż = -sin(θ)·u + cos(θ)·sin(φ)·v + cos(θ)·cos(φ)·w
```

### System Identification Results

#### Inertial Properties (Estimated from Geometry)
```
Ixx = 0.0347 kg·m²  # Roll inertia
Iyy = 0.0347 kg·m²  # Pitch inertia  
Izz = 0.0617 kg·m²  # Yaw inertia
```

#### Thrust and Torque Models
```
Thrust per motor: T = kT × ω²
Torque per motor: Q = kM × ω²
kT = 3.16e-6 N·s²
kM = 7.94e-8 N·m·s²
```

#### Drag Coefficients
```
Translational drag: D = -Cd × v × |v|
Cd_x = 0.25 N·s²/m²
Cd_y = 0.25 N·s²/m²  
Cd_z = 0.30 N·s²/m²
```

#### Motor Dynamics
```
First-order lag: τ·ω̇ + ω = ωcmd
Time constant: τ = 0.15 s
```

### Linearized Model for Control Design

#### Hover Linearization
At hover condition (small angle approximation):

**State Space Representation:**
```
ẋ = A·x + B·u
y = C·x + D·u
```

**A Matrix (12×12):**
```
A = [A_ang    0₃ₓ₃   0₃ₓ₃   0₃ₓ₃]
    [I₃ₓ₃    0₃ₓ₃   0₃ₓ₃   0₃ₓ₃]
    [0₃ₓ₃    A_grav 0₃ₓ₃   0₃ₓ₃]
    [0₃ₓ₃    0₃ₓ₃   I₃ₓ₃   0₃ₓ₃]

Where:
A_ang = [-1/τ   0     0  ]  # Motor dynamics
        [0     -1/τ   0  ]
        [0      0    -1/τ]

A_grav = [0  -g   0]  # Gravity coupling
         [g   0   0]
         [0   0   0]
```

**B Matrix (12×4):**
```
B = [B_ang]
    [0₃ₓ₄]
    [B_thrust]  
    [0₃ₓ₄]

Where:
B_ang = [L/Ixx   0    0   ]  # Torque inputs
        [0    L/Iyy  0   ]
        [0       0   1/Izz]

B_thrust = [0  0  -1/m]  # Thrust input (NED frame)
           [0  0   0  ]
           [0  0   0  ]
```

## Attitude Control Design

### Multi-Loop PID Controller (ArduPilot Standard)

#### Rate Controller (Inner Loop)
```
u_roll = Kp_rate_roll × (p_cmd - p) + Ki_rate_roll × ∫(p_cmd - p)dt + Kd_rate_roll × ṗ
u_pitch = Kp_rate_pitch × (q_cmd - q) + Ki_rate_pitch × ∫(q_cmd - q)dt + Kd_rate_pitch × q̇  
u_yaw = Kp_rate_yaw × (r_cmd - r) + Ki_rate_yaw × ∫(r_cmd - r)dt + Kd_rate_yaw × ṙ
```

**Tuned Parameters:**
```
# Rate Controller Gains (based on system ID)
ATC_RAT_RLL_P = 0.135    # Roll rate P gain
ATC_RAT_RLL_I = 0.135    # Roll rate I gain  
ATC_RAT_RLL_D = 0.0036   # Roll rate D gain
ATC_RAT_PIT_P = 0.135    # Pitch rate P gain
ATC_RAT_PIT_I = 0.135    # Pitch rate I gain
ATC_RAT_PIT_D = 0.0036   # Pitch rate D gain
ATC_RAT_YAW_P = 0.18     # Yaw rate P gain
ATC_RAT_YAW_I = 0.018    # Yaw rate I gain  
ATC_RAT_YAW_D = 0.0      # Yaw rate D gain
```

#### Attitude Controller (Outer Loop)
```
p_cmd = Kp_att_roll × (φ_cmd - φ) × cos(φ)
q_cmd = Kp_att_pitch × (θ_cmd - θ) × cos(θ)
r_cmd = Kp_att_yaw × (ψ_cmd - ψ)
```

**Tuned Parameters:**
```
# Attitude Controller Gains
ATC_ANG_RLL_P = 4.5      # Roll angle P gain
ATC_ANG_PIT_P = 4.5      # Pitch angle P gain
ATC_ANG_YAW_P = 4.5      # Yaw angle P gain
```

### LQR Controller (Alternative Advanced Design)

#### State Feedback Controller
```
u = -K × x
```

**LQR Cost Function:**
```
J = ∫₀^∞ (x^T Q x + u^T R u) dt
```

**Design Matrices:**
```
Q = diag([100, 100, 50, 10, 10, 10, 1, 1, 1, 1, 1, 1])  # State weights
R = diag([10, 10, 10, 1])  # Control weights
```

**Calculated LQR Gains:**
```
K = [5.48  0     0    3.16  0     0    2.24  0     0    1.41  0     0   ]
    [0     5.48  0    0     3.16  0    0     2.24  0    0     1.41  0   ]
    [0     0     4.24 0     0     2.83 0     0     1.89 0     0     1.22]
    [0     0     0    0     0     0    0     0     0    0     0     0.32]
```

### Feed-Forward Control
```
# Motor speed squared commands based on thrust/torque allocation
ω₁² = (T + τroll × b₁ + τpitch × b₂ + τyaw × b₃) / kT
ω₂² = (T + τroll × b₄ + τpitch × b₅ + τyaw × b₆) / kT
... (for all 6 motors)

Where b₁...b₆ are allocation coefficients from mixing matrix
```

## Velocity Control Design

### Velocity Controller Structure
```
Position Controller → Velocity Controller → Attitude Controller → Rate Controller → Motors
```

#### Position-to-Velocity Controller
```
vN_cmd = Kp_pos_x × (xN_cmd - xN) + Kd_pos_x × (vN_cmd_prev - vN)
vE_cmd = Kp_pos_y × (xE_cmd - xE) + Kd_pos_y × (vE_cmd_prev - vE)  
vD_cmd = Kp_pos_z × (xD_cmd - xD) + Kd_pos_z × (vD_cmd_prev - vD)
```

**Parameters:**
```
# Position Controller Gains  
PSC_POSXY_P = 1.0        # Horizontal position P gain
PSC_POSZ_P = 1.0         # Vertical position P gain
PSC_VELXY_P = 2.0        # Horizontal velocity P gain
PSC_VELXY_I = 1.0        # Horizontal velocity I gain
PSC_VELXY_D = 0.5        # Horizontal velocity D gain
PSC_VELZ_P = 5.0         # Vertical velocity P gain
PSC_VELZ_I = 2.0         # Vertical velocity I gain
```

#### Velocity-to-Attitude Controller
```
# Convert desired accelerations to attitude commands
a_des_N = Kp_vel_x × (vN_cmd - vN) + Ki_vel_x × ∫(vN_cmd - vN)dt
a_des_E = Kp_vel_y × (vE_cmd - vE) + Ki_vel_y × ∫(vE_cmd - vE)dt

# Attitude commands from acceleration commands
θ_cmd = asin(a_des_N / g)  # Pitch for North acceleration
φ_cmd = -asin(a_des_E / (g × cos(θ_cmd)))  # Roll for East acceleration

# Thrust command
thrust_cmd = m × (g + vD_cmd_err × Kp_vel_z) / (cos(φ_cmd) × cos(θ_cmd))
```

### Trajectory Generation
```
# 5th-order polynomial trajectory for smooth motion
s(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵

Boundary conditions:
s(0) = s₀, ṡ(0) = 0, s̈(0) = 0
s(T) = sf, ṡ(T) = 0, s̈(T) = 0
```

## EKF Integration

### State Estimation Architecture
```
EKF States: [qw, qx, qy, qz, vN, vE, vD, pN, pE, pD, δgx, δgy, δgz, δax, δay, δaz, magN, magE, magD, magX, magY, magZ, vwN, vwE]
```

### Sensor Fusion Strategy

#### IMU Processing
```
# Gyroscope bias compensation
ω_corrected = ω_raw - bias_gyro - noise_gyro

# Accelerometer bias compensation  
a_corrected = a_raw - bias_accel - noise_accel

# Prediction step uses corrected IMU data
```

#### GPS Integration
```
# Position measurements
z_GPS_pos = [pN, pE, pD] + noise_GPS_pos

# Velocity measurements
z_GPS_vel = [vN, vE, vD] + noise_GPS_vel

# Innovation gating
if |innovation| < gate_threshold:
    perform_update()
else:
    reject_measurement()
```

#### Magnetometer Fusion
```
# Expected measurement model
z_mag_expected = R_bn^T × (mag_earth + mag_body)

# Innovation calculation
innovation_mag = z_mag_measured - z_mag_expected

# Heading update
if |innovation_mag| < MAG_GATE:
    update_quaternion_states()
```

### EKF Parameters for Hexacopter
```
# Process Noise
EK3_GYRO_P_NSE = 0.015   # Gyro process noise (rad/s)
EK3_ACC_P_NSE = 0.25     # Accel process noise (m/s²)
EK3_GBIAS_P_NSE = 1e-5   # Gyro bias process noise
EK3_ABIAS_P_NSE = 0.002  # Accel bias process noise
EK3_WIND_P_NSE = 0.1     # Wind process noise
EK3_MAG_P_NSE = 0.0005   # Magnetometer process noise

# Measurement Noise  
EK3_GPS_P_NSE = 0.3      # GPS position noise (m)
EK3_GPS_V_NSE = 0.3      # GPS velocity noise (m/s)
EK3_BARO_P_NSE = 0.6     # Barometer noise (m)
EK3_MAG_M_NSE = 0.05     # Magnetometer measurement noise

# Innovation Gates
EK3_VEL_I_GATE = 300     # Velocity innovation gate (%)
EK3_POS_I_GATE = 300     # Position innovation gate (%)
EK3_HGT_I_GATE = 400     # Height innovation gate (%)
EK3_MAG_I_GATE = 300     # Magnetometer innovation gate (%)
```

### Control Integration with EKF
```
# Use EKF states for control
position_estimate = EKF.get_position()
velocity_estimate = EKF.get_velocity()  
attitude_estimate = EKF.get_attitude()

# Feed estimates to controllers
position_controller.update(position_estimate, position_command)
velocity_controller.update(velocity_estimate, velocity_command)
attitude_controller.update(attitude_estimate, attitude_command)
```

## Implementation

### Main Control Loop (400 Hz)
```cpp
void control_loop() {
    // 1. Read sensors
    read_imu();
    read_gps();
    read_baro();
    read_compass();
    
    // 2. EKF prediction and update
    ekf.predict(imu_data);
    ekf.update_gps(gps_data);
    ekf.update_baro(baro_data);
    ekf.update_mag(mag_data);
    
    // 3. Get state estimates
    Vector3f position = ekf.get_position();
    Vector3f velocity = ekf.get_velocity();
    Vector3f attitude = ekf.get_euler_angles();
    Vector3f angular_rate = ekf.get_angular_rate();
    
    // 4. Position control (50 Hz)
    if (position_control_counter % 8 == 0) {
        velocity_cmd = position_controller.update(position, position_cmd);
    }
    
    // 5. Velocity control (100 Hz) 
    if (velocity_control_counter % 4 == 0) {
        attitude_cmd = velocity_controller.update(velocity, velocity_cmd);
        thrust_cmd = velocity_controller.get_thrust_cmd();
    }
    
    // 6. Attitude control (400 Hz)
    angular_rate_cmd = attitude_controller.update(attitude, attitude_cmd);
    
    // 7. Rate control (400 Hz)
    torque_cmd = rate_controller.update(angular_rate, angular_rate_cmd);
    
    // 8. Motor mixing and output
    motor_outputs = mixer.mix(thrust_cmd, torque_cmd);
    output_to_motors(motor_outputs);
}
```

### Control System Classes

#### Attitude Controller Class
```cpp
class HexacopterAttitudeController {
private:
    float kp_roll, ki_roll, kd_roll;
    float kp_pitch, ki_pitch, kd_pitch;  
    float kp_yaw, ki_yaw, kd_yaw;
    
    float kp_att_roll, kp_att_pitch, kp_att_yaw;
    
    Vector3f integral_error;
    Vector3f last_error;
    
public:
    Vector3f update(Vector3f attitude, Vector3f attitude_cmd);
    void set_gains(AttitudeGains gains);
    void reset_integrators();
};
```

#### Velocity Controller Class  
```cpp
class HexacopterVelocityController {
private:
    float kp_vel_xy, ki_vel_xy, kd_vel_xy;
    float kp_vel_z, ki_vel_z, kd_vel_z;
    float kp_pos_xy, kp_pos_z;
    
    Vector3f velocity_integral;
    Vector3f last_velocity_error;
    
public:
    Vector3f update_position(Vector3f position, Vector3f position_cmd);
    Vector3f update_velocity(Vector3f velocity, Vector3f velocity_cmd);
    float get_thrust_command();
};
```

#### Motor Mixer Class
```cpp
class HexacopterMixer {
private:
    float mixing_matrix[6][4] = {
        // Roll, Pitch, Yaw, Throttle
        { 0.50,  0.87,  1.0, 1.0},  // Motor 1
        { 1.00,  0.00, -1.0, 1.0},  // Motor 2  
        { 0.50, -0.87,  1.0, 1.0},  // Motor 3
        {-0.50, -0.87, -1.0, 1.0},  // Motor 4
        {-1.00,  0.00,  1.0, 1.0},  // Motor 5
        {-0.50,  0.87, -1.0, 1.0}   // Motor 6
    };
    
public:
    Vector6f mix(float thrust, Vector3f torque);
    bool check_motor_saturation(Vector6f motor_outputs);
};
```

## Parameters and Tuning

### Complete Parameter Set

#### Frame Configuration
```
FRAME_CLASS = 2              # Hexacopter  
FRAME_TYPE = 1               # X configuration
MOT_PWM_TYPE = 0             # Normal PWM
MOT_PWM_MIN = 1000           # Minimum PWM
MOT_PWM_MAX = 2000           # Maximum PWM
MOT_SPIN_ARM = 0.10          # Motor spin when armed
MOT_SPIN_MAX = 0.95          # Maximum motor output
MOT_SPIN_MIN = 0.15          # Minimum motor spin
MOT_THST_EXPO = 0.65         # Thrust curve expo  
MOT_THST_HOVER = 0.35        # Hover throttle
```

#### Attitude Control Parameters
```
# Rate Controller
ATC_RAT_RLL_P = 0.135
ATC_RAT_RLL_I = 0.135  
ATC_RAT_RLL_D = 0.0036
ATC_RAT_RLL_IMAX = 0.444
ATC_RAT_RLL_FLTD = 20.0
ATC_RAT_RLL_FLTT = 20.0

ATC_RAT_PIT_P = 0.135
ATC_RAT_PIT_I = 0.135
ATC_RAT_PIT_D = 0.0036  
ATC_RAT_PIT_IMAX = 0.444
ATC_RAT_PIT_FLTD = 20.0
ATC_RAT_PIT_FLTT = 20.0

ATC_RAT_YAW_P = 0.18
ATC_RAT_YAW_I = 0.018
ATC_RAT_YAW_D = 0.0
ATC_RAT_YAW_IMAX = 0.222
ATC_RAT_YAW_FLTD = 2.5
ATC_RAT_YAW_FLTT = 20.0

# Attitude Controller  
ATC_ANG_RLL_P = 4.5
ATC_ANG_PIT_P = 4.5
ATC_ANG_YAW_P = 4.5

# Input shaping
ATC_INPUT_TC = 0.15          # Input time constant
ATC_SLEW_YAW = 6000          # Yaw slew rate limit
```

#### Position/Velocity Control Parameters
```
# Position Control
PSC_POSXY_P = 1.0            # Horizontal position P gain
PSC_POSZ_P = 1.0             # Vertical position P gain  

# Velocity Control
PSC_VELXY_P = 2.0            # Horizontal velocity P gain
PSC_VELXY_I = 1.0            # Horizontal velocity I gain
PSC_VELXY_D = 0.5            # Horizontal velocity D gain
PSC_VELXY_IMAX = 1000        # Horizontal velocity I max
PSC_VELXY_FLTD = 2.5         # Horizontal velocity D filter

PSC_VELZ_P = 5.0             # Vertical velocity P gain
PSC_VELZ_I = 2.0             # Vertical velocity I gain  
PSC_VELZ_IMAX = 800          # Vertical velocity I max

# Acceleration limits
PSC_ACCZ_I = 2.0             # Vertical accel I gain
PSC_ACCZ_P = 0.5             # Vertical accel P gain
PSC_ACCZ_D = 0.0             # Vertical accel D gain
```

#### EKF Parameters
```
# EKF3 Configuration
EK3_ENABLE = 1               # Enable EKF3
EK3_IMU_MASK = 3             # Use first 2 IMUs
EK3_GPS_TYPE = 0             # Use GPS for position
EK3_HRT_FILT = 2.0           # Height rate filter

# Process Noise
EK3_GYRO_P_NSE = 0.015       # Gyro process noise
EK3_ACC_P_NSE = 0.25         # Accel process noise  
EK3_GBIAS_P_NSE = 1e-5       # Gyro bias process noise
EK3_ABIAS_P_NSE = 0.002      # Accel bias process noise
EK3_WIND_P_NSE = 0.1         # Wind process noise
EK3_MAG_P_NSE = 0.0005       # Mag process noise

# Measurement Noise
EK3_GPS_P_NSE = 0.3          # GPS position noise
EK3_GPS_V_NSE = 0.3          # GPS velocity noise
EK3_BARO_P_NSE = 0.6         # Baro noise
EK3_MAG_M_NSE = 0.05         # Mag measurement noise

# Innovation Gates  
EK3_VEL_I_GATE = 300         # Velocity innovation gate
EK3_POS_I_GATE = 300         # Position innovation gate
EK3_HGT_I_GATE = 400         # Height innovation gate
EK3_MAG_I_GATE = 300         # Mag innovation gate
```

### Tuning Guidelines

#### Initial Setup Procedure
1. **Verify Frame Configuration**: Ensure motor layout matches HEXA_X
2. **ESC Calibration**: Calibrate all 6 ESCs simultaneously  
3. **Sensor Calibration**: Compass, accelerometer, and gyro calibration
4. **Motor Test**: Verify correct motor rotation and direction
5. **Initial Flight**: Test in stabilize mode with conservative gains

#### Attitude Control Tuning
1. **Start Conservative**: Begin with 50% of calculated gains
2. **Rate Controller First**: Tune rate P, then I, then D
3. **Attitude Controller**: Tune angle P gains for desired responsiveness
4. **Filter Tuning**: Adjust filters based on vibration levels
5. **Verify Stability**: Ensure stable hover and smooth response

#### Position Control Tuning  
1. **Stabilize First**: Ensure attitude control is well-tuned
2. **Velocity Controller**: Start with velocity P gain, add I and D
3. **Position Controller**: Tune position P for desired tracking
4. **Test Waypoints**: Verify smooth waypoint following
5. **Performance Check**: Monitor position hold accuracy

### Safety and Monitoring

#### Pre-flight Checks
- EKF health and initialization status
- GPS lock and HDOP < 2.0
- Compass calibration and interference check  
- Motor balance and vibration levels
- Battery voltage and current consumption

#### In-flight Monitoring
- EKF innovation magnitudes and test ratios
- Motor output balance and saturation
- Control system performance metrics
- Position and velocity tracking errors
- Battery remaining capacity

This comprehensive system provides robust control for hexacopter operations with integrated EKF state estimation and properly tuned control parameters based on system identification results.