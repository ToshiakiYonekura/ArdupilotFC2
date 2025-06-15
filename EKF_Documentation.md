# Extended Kalman Filter (EKF) Documentation for ArduPilot

## Table of Contents
1. [Introduction](#introduction)
2. [EKF Theory and Mathematical Foundation](#ekf-theory-and-mathematical-foundation)
3. [EKF Overview](#ekf-overview)
4. [ArduPilot EKF Implementation](#ardupilot-ekf-implementation)
5. [EKF3 - Primary Navigation Filter](#ekf3---primary-navigation-filter)
6. [EKF2 - Legacy Navigation Filter](#ekf2---legacy-navigation-filter)
7. [Specialized EKF Applications](#specialized-ekf-applications)
8. [State Vector and System Model](#state-vector-and-system-model)
9. [Sensor Fusion](#sensor-fusion)
10. [Configuration and Parameters](#configuration-and-parameters)
11. [Performance and Tuning](#performance-and-tuning)
12. [Troubleshooting](#troubleshooting)
13. [Advanced Topics](#advanced-topics)

## Introduction

The Extended Kalman Filter (EKF) is a fundamental component of ArduPilot's navigation system, providing optimal state estimation by fusing data from multiple sensors including IMU (Inertial Measurement Unit), GPS, magnetometer, barometer, optical flow, and other external sensors. This documentation provides a comprehensive overview of EKF implementation in ArduPilot, covering both theoretical foundations and practical implementation details.

## EKF Theory and Mathematical Foundation

### Historical Background

The Kalman filter was developed by Rudolf Kalman in 1960 as an optimal recursive solution to the linear filtering problem. The Extended Kalman Filter (EKF) was later developed to handle non-linear systems by linearizing the system equations around the current state estimate.

#### Key Mathematical Concepts:

1. **Bayesian Estimation**: EKF provides the optimal estimate in the minimum mean square error sense
2. **Recursive Processing**: New estimates are computed from previous estimates plus new measurements
3. **Gaussian Assumption**: States and measurement noise are assumed to be Gaussian distributed
4. **Linearization**: Non-linear functions are approximated using first-order Taylor expansions

### The Linear Kalman Filter Foundation

Before understanding the EKF, it's essential to understand the linear Kalman filter which forms its foundation.

#### System Model:
```
State Evolution:      x(k+1) = F(k)·x(k) + G(k)·u(k) + w(k)
Measurement Model:    z(k) = H(k)·x(k) + v(k)
```

Where:
- `x(k)`: State vector at time k
- `F(k)`: State transition matrix
- `G(k)`: Control input matrix
- `u(k)`: Control input vector
- `w(k)`: Process noise ~ N(0, Q(k))
- `z(k)`: Measurement vector
- `H(k)`: Measurement matrix
- `v(k)`: Measurement noise ~ N(0, R(k))

#### Kalman Filter Algorithm:

**1. Prediction Step:**
```
x̂(k|k-1) = F(k-1)·x̂(k-1|k-1) + G(k-1)·u(k-1)
P(k|k-1) = F(k-1)·P(k-1|k-1)·F(k-1)ᵀ + Q(k-1)
```

**2. Update Step:**
```
Innovation:     ỹ(k) = z(k) - H(k)·x̂(k|k-1)
Innovation Cov: S(k) = H(k)·P(k|k-1)·H(k)ᵀ + R(k)
Kalman Gain:    K(k) = P(k|k-1)·H(k)ᵀ·S(k)⁻¹
State Update:   x̂(k|k) = x̂(k|k-1) + K(k)·ỹ(k)
Covar Update:   P(k|k) = (I - K(k)·H(k))·P(k|k-1)
```

### Extended Kalman Filter Theory

The EKF extends the Kalman filter to non-linear systems by linearizing the non-linear functions.

#### Non-Linear System Model:
```
State Evolution:      x(k+1) = f(x(k), u(k), k) + w(k)
Measurement Model:    z(k) = h(x(k), k) + v(k)
```

Where:
- `f(·)`: Non-linear state transition function
- `h(·)`: Non-linear measurement function

#### Linearization Process:

The EKF approximates non-linear functions using first-order Taylor series expansion:

**State Transition Jacobian:**
```
F(k) = ∂f/∂x |_{x=x̂(k|k),u=u(k)}
```

**Measurement Jacobian:**
```
H(k) = ∂h/∂x |_{x=x̂(k|k-1)}
```

#### EKF Algorithm:

**1. Prediction Step:**
```
x̂(k|k-1) = f(x̂(k-1|k-1), u(k-1), k-1)
F(k-1) = ∂f/∂x |_{x=x̂(k-1|k-1),u=u(k-1)}
P(k|k-1) = F(k-1)·P(k-1|k-1)·F(k-1)ᵀ + Q(k-1)
```

**2. Update Step:**
```
ẑ(k|k-1) = h(x̂(k|k-1), k)
H(k) = ∂h/∂x |_{x=x̂(k|k-1)}
ỹ(k) = z(k) - ẑ(k|k-1)
S(k) = H(k)·P(k|k-1)·H(k)ᵀ + R(k)
K(k) = P(k|k-1)·H(k)ᵀ·S(k)⁻¹
x̂(k|k) = x̂(k|k-1) + K(k)·ỹ(k)
P(k|k) = (I - K(k)·H(k))·P(k|k-1)
```

### Navigation-Specific Mathematical Models

For aerial vehicle navigation, the EKF must handle several key mathematical relationships:

#### 1. Quaternion Dynamics

Quaternions provide a singularity-free representation of attitude:

```
Quaternion: q = [q₀, q₁, q₂, q₃]ᵀ = [qw, qx, qy, qz]ᵀ
Constraint: |q| = √(qw² + qx² + qy² + qz²) = 1

Quaternion Kinematic Equation:
q̇ = (1/2) × Ω(ω) × q

Where Ω(ω) is the skew-symmetric matrix:
Ω(ω) = [  0   -ωx  -ωy  -ωz ]
       [ ωx    0    ωz  -ωy ]
       [ ωy   -ωz   0    ωx ]
       [ ωz    ωy  -ωx   0  ]
```

#### 2. Rotation Matrix from Quaternion

```
R(q) = I + 2qw[q×] + 2[q×]²

Where [q×] is the skew-symmetric matrix of q = [qx, qy, qz]ᵀ:
[q×] = [  0   -qz   qy ]
       [ qz    0   -qx ]
       [-qy   qx    0  ]

Expanded form:
R(q) = [ 1-2(qy²+qz²)   2(qxqy-qwqz)   2(qxqz+qwqy) ]
       [ 2(qxqy+qwqz)   1-2(qx²+qz²)   2(qyqz-qwqx) ]
       [ 2(qxqz-qwqy)   2(qyqz+qwqx)   1-2(qx²+qy²) ]
```

#### 3. Navigation Equations

The navigation equations relate body-frame accelerations to navigation-frame motion:

```
Position Dynamics:
ṗN = vN    (North position rate)
ṗE = vE    (East position rate)  
ṗD = vD    (Down position rate)

Velocity Dynamics:
v̇N = aN + (vE·ωD - vD·ωE)    (North velocity rate)
v̇E = aE + (vD·ωN - vN·ωD)    (East velocity rate)
v̇D = aD + g + (vN·ωE - vE·ωN) (Down velocity rate)

Where:
- aN, aE, aD: Navigation frame accelerations
- ωN, ωE, ωD: Earth rotation and transport rate components
- g: Gravity magnitude
```

#### 4. Sensor Error Models

**IMU Error Models:**
```
Gyroscope Model:
ωmeas = ωtrue + bgyr + ngyr + scale_error × ωtrue

Accelerometer Model:
ameas = atrue + bacc + nacc + scale_error × atrue

Where:
- bgyr, bacc: Sensor biases (modeled as random walk)
- ngyr, nacc: White noise
- scale_error: Scale factor errors
```

**GPS Measurement Model:**
```
Position: zGPS_pos = p + nGPS_pos
Velocity: zGPS_vel = v + nGPS_vel

Where nGPS represents GPS measurement noise
```

**Magnetometer Model:**
```
zmag = R(q)ᵀ × (magearth + magbody) + nmag

Where:
- magearth: Earth's magnetic field in navigation frame
- magbody: Body-frame magnetic disturbances
- R(q)ᵀ: Transformation from navigation to body frame
```

### Statistical Foundation

#### Gaussian Distribution Properties:

The EKF relies heavily on Gaussian distribution properties:

```
Probability Density Function:
p(x) = (1/√(2π|P|)) × exp(-½(x-μ)ᵀP⁻¹(x-μ))

Where:
- μ: Mean vector
- P: Covariance matrix
- |P|: Determinant of P
```

#### Covariance Propagation:

For a non-linear transformation y = f(x) where x ~ N(μ, P):

```
First-order approximation:
μy ≈ f(μx)
Py ≈ F × Px × Fᵀ

Where F = ∂f/∂x |_{x=μx}
```

### Innovation Sequence Theory

The innovation sequence provides crucial information about filter performance:

#### Innovation Properties:
```
Innovation: ỹ(k) = z(k) - h(x̂(k|k-1))
Expected Value: E[ỹ(k)] = 0 (for optimal filter)
Covariance: S(k) = H(k)·P(k|k-1)·H(k)ᵀ + R(k)
```

#### Normalized Innovation Squared (NIS):
```
NIS(k) = ỹ(k)ᵀ × S(k)⁻¹ × ỹ(k)

For optimal filter: NIS ~ χ²(m) where m is measurement dimension
```

#### Innovation-Based Tests:
- **Whiteness Test**: Innovations should be uncorrelated over time
- **Zero-Mean Test**: Innovation mean should be zero
- **Consistency Test**: NIS should follow chi-squared distribution

### Observability and Controllability

#### Observability Analysis:

For the linear case, the observability matrix is:
```
O = [H, HF, HF², ..., HF^(n-1)]ᵀ

System is observable if rank(O) = n (state dimension)
```

For navigation systems, observability depends on:
- **Motion Dynamics**: Vehicle must be maneuvering for full observability
- **Sensor Geometry**: Multiple independent measurements improve observability
- **Time Correlation**: Extended observation time improves weak observabilities

#### Common Observability Issues:
1. **Yaw Unobservable**: During straight-line motion without magnetometer
2. **Scale Unobservable**: In visual odometry without external reference
3. **Bias Unobservable**: IMU biases require accelerations to become observable

### Numerical Considerations

#### Covariance Matrix Properties:
```
Symmetry: P = Pᵀ
Positive Semi-Definite: xᵀPx ≥ 0 for all x
```

#### Joseph Form Update:
To maintain numerical stability and positive definiteness:
```
P(k|k) = (I - K(k)H(k))P(k|k-1)(I - K(k)H(k))ᵀ + K(k)R(k)K(k)ᵀ
```

#### Square Root Filtering:
For improved numerical stability, factorize P = S·Sᵀ and propagate S instead of P.

### Limitations and Assumptions

#### EKF Assumptions:
1. **Gaussian Noise**: Process and measurement noise are Gaussian
2. **Linear Approximation**: First-order Taylor expansion is adequate
3. **Known Models**: Process and measurement models are known
4. **White Noise**: Noise sequences are uncorrelated

#### Common Failure Modes:
1. **Linearization Errors**: When higher-order terms are significant
2. **Model Mismatch**: When assumed models don't match reality
3. **Numerical Instability**: When covariance matrix loses positive definiteness
4. **Divergence**: When linearization point becomes too far from true state

### Alternative Approaches

#### Unscented Kalman Filter (UKF):
Uses sigma points instead of linearization to handle non-linearities:
```
Sigma Points: Xi = x̂ ± √((n+λ)P)
Weights: Wi^m, Wi^c for mean and covariance
```

#### Particle Filter:
Uses Monte Carlo methods to represent arbitrary probability distributions:
```
Particles: {xi^(j), wi^(j)}j=1^N
Resampling: Based on importance weights
```

#### Factor Graph Optimization:
Formulates SLAM as a factor graph optimization problem:
```
Minimize: Σ ||fi(xi) - zi||²Σi^-1
Subject to: Vehicle dynamics constraints
```

## EKF Overview

### What is an Extended Kalman Filter?

The Extended Kalman Filter is a recursive state estimator that extends the linear Kalman filter to handle non-linear systems. It operates on the principle of predicting system states based on a mathematical model and then correcting these predictions using sensor measurements.

#### Key Concepts:

1. **State Estimation**: Estimates the vehicle's position, velocity, attitude, and sensor biases
2. **Sensor Fusion**: Combines measurements from multiple sensors optimally
3. **Uncertainty Quantification**: Tracks estimation uncertainty through covariance matrices
4. **Recursive Processing**: Updates estimates continuously as new measurements arrive

#### EKF Process:

1. **Prediction Step**: Uses IMU data and system dynamics to predict state evolution
2. **Correction Step**: Updates predictions using external sensor measurements (GPS, magnetometer, etc.)
3. **Innovation Analysis**: Evaluates measurement consistency and filter health

## ArduPilot EKF Implementation

ArduPilot implements multiple EKF variants to provide robust navigation across different vehicle types and mission requirements:

### File Structure:
```
libraries/
├── AP_NavEKF3/          # Primary navigation filter (24-state)
├── AP_NavEKF2/          # Legacy navigation filter (24-state)
├── AP_NavEKF/           # Common base classes and utilities
└── AP_Soaring/          # Specialized EKF for thermal detection
    └── ExtendedKalmanFilter.cpp
```

### Core Components:

1. **Frontend Classes**: `NavEKF3`, `NavEKF2` - Manage multiple filter cores
2. **Core Classes**: `NavEKF3_core`, `NavEKF2_core` - Implement filter algorithms
3. **Common Utilities**: Shared data structures, buffers, and interfaces

## EKF3 - Primary Navigation Filter

EKF3 is the current generation navigation filter, based on Paul Riseborough's 24-state derivation from the PX4/ECL project.

### Key Features:

- **24-State Vector**: Comprehensive state representation
- **Multi-Core Support**: Up to 6 parallel filter instances for redundancy
- **Advanced Sensor Fusion**: Supports modern sensors and external navigation
- **Robust Fault Detection**: Comprehensive health monitoring and failover

### State Vector (24 States):

```cpp
struct state_elements {
    QuaternionF quat;           // Quaternion (4 states) - Body to NED rotation
    Vector3F    velocity;       // Velocity NED (3 states) - m/s
    Vector3F    position;       // Position NED (3 states) - m
    Vector3F    gyro_bias;      // Gyro bias (3 states) - rad/s
    Vector3F    accel_bias;     // Accel bias (3 states) - m/s²
    Vector3F    earth_magfield; // Earth magnetic field (3 states) - Gauss
    Vector3F    body_magfield;  // Body magnetic field (3 states) - Gauss
    Vector2F    wind_vel;       // Wind velocity NE (2 states) - m/s
};
```

### Sensor Support:

- **IMU**: Primary motion sensing (gyroscopes, accelerometers)
- **GPS**: Position, velocity, and heading measurements
- **Magnetometer**: Heading reference and magnetic field estimation
- **Barometer**: Altitude reference
- **Range Finder**: Terrain-relative altitude
- **Optical Flow**: Visual odometry for GPS-denied navigation
- **External Navigation**: VICON, T265 camera, wheel odometry
- **Airspeed**: True airspeed for fixed-wing aircraft

### Multi-Core Architecture:

EKF3 runs multiple filter cores simultaneously:
- **Primary Core**: Main navigation solution
- **Secondary Cores**: Backup solutions with different sensor configurations
- **Core Selection**: Automatic switching based on innovation consistency
- **Lane Switching**: Seamless transitions between healthy cores

## EKF2 - Legacy Navigation Filter

EKF2 is the previous generation filter, still maintained for compatibility and specific use cases.

### Key Differences from EKF3:

- **Simpler Architecture**: Single or dual core operation
- **Different Derivation**: Based on earlier mathematical formulation
- **Legacy Compatibility**: Supports older parameter sets and configurations
- **Reduced Complexity**: Fewer advanced features but proven stability

### When to Use EKF2:

- Legacy systems requiring parameter compatibility
- Applications needing simpler configuration
- Hardware with limited computational resources
- Specific mission requirements validated with EKF2

## Specialized EKF Applications

### Soaring EKF

Located in `AP_Soaring/ExtendedKalmanFilter.cpp`, this is a specialized 4-state EKF for thermal detection:

```cpp
class ExtendedKalmanFilter {
    static constexpr const uint8_t N = 4;
    VectorN<float,N> X;      // State vector (thermal parameters)
    MatrixN<float,N> P;      // Covariance matrix
    MatrixN<float,N> Q;      // Process noise
    float R;                 // Measurement noise
};
```

**Applications:**
- Thermal detection and mapping for gliders
- Updraft strength estimation
- Optimal soaring path planning

## State Vector and System Model

### Coordinate Frames:

1. **Body Frame**: Vehicle-fixed coordinate system
   - X: Forward, Y: Right, Z: Down
   - Used for IMU measurements and control

2. **NED Frame**: North-East-Down navigation frame
   - X: North, Y: East, Z: Down
   - Local tangent plane to Earth's surface

3. **Earth Frame**: Earth-Centered Earth-Fixed (ECEF)
   - Used for GPS measurements and global positioning

### State Dynamics:

The EKF propagates states using differential equations:

```
Quaternion:     q̇ = 0.5 × q ⊗ ω
Velocity:       v̇ = R(q) × a + g
Position:       ṗ = v
Gyro Bias:      ḃω = noise
Accel Bias:     ḃa = noise
Mag Fields:     ṁ = noise (slowly varying)
Wind:           ẇ = noise
```

Where:
- `q`: Quaternion
- `ω`: Angular velocity (gyro - bias)
- `R(q)`: Rotation matrix from quaternion
- `a`: Specific force (accel - bias)
- `g`: Gravity vector

### Process Noise:

Each state has associated process noise to model uncertainties:
- **IMU Biases**: Random walk processes
- **Magnetic Fields**: Slowly varying with environment
- **Wind**: Varying atmospheric conditions
- **Attitude/Position**: Driven by IMU measurement noise

## Sensor Fusion

### Measurement Models:

Each sensor provides measurements related to the state vector:

#### GPS Measurements:
```
Position:   z_pos = p + measurement_noise
Velocity:   z_vel = v + measurement_noise
```

#### Magnetometer Measurements:
```
z_mag = R(q)ᵀ × (earth_mag + body_mag) + noise
```

#### Barometer Measurements:
```
z_baro = -p_down + bias + noise
```

#### Optical Flow Measurements:
```
z_flow = predicted_flow_rate(v, ω, height) + noise
```

### Innovation Analysis:

The EKF continuously monitors measurement innovations (difference between predicted and actual measurements):

```
Innovation = measurement - prediction
Innovation_variance = H × P × Hᵀ + R
Normalized_innovation = innovation² / innovation_variance
```

- **Consistency Checks**: Reject measurements with excessive innovations
- **Health Monitoring**: Track filter performance and sensor quality
- **Fault Detection**: Identify sensor failures and system issues

## Configuration and Parameters

### Key EKF3 Parameters:

#### Basic Configuration:
- `EK3_ENABLE`: Enable/disable EKF3 (0/1)
- `EK3_IMU_MASK`: Bitmask of IMUs to use
- `EK3_PRIMARY`: Preferred primary core
- `EK3_GPS_TYPE`: GPS fusion mode

#### Noise Parameters:
- `EK3_GYRO_P_NSE`: Gyro process noise (rad/s)
- `EK3_ACC_P_NSE`: Accelerometer process noise (m/s²)
- `EK3_GBIAS_P_NSE`: Gyro bias process noise
- `EK3_ABIAS_P_NSE`: Accel bias process noise
- `EK3_MAG_P_NSE`: Magnetometer process noise

#### Measurement Noise:
- `EK3_GPS_P_NSE`: GPS position noise (m)
- `EK3_GPS_V_NSE`: GPS velocity noise (m/s)
- `EK3_BARO_P_NSE`: Barometer noise (m)
- `EK3_MAG_M_NSE`: Magnetometer measurement noise

#### Innovation Gates:
- `EK3_POSNE_M_NSE`: Position innovation gate
- `EK3_VELD_M_NSE`: Velocity innovation gate
- `EK3_VEL_I_GATE`: Velocity innovation gate percentage
- `EK3_POS_I_GATE`: Position innovation gate percentage

### Typical Parameter Sets:

#### Default Configuration:
```
EK3_ENABLE = 1
EK3_IMU_MASK = 3
EK3_GPS_TYPE = 0
EK3_GYRO_P_NSE = 0.015
EK3_ACC_P_NSE = 0.25
```

#### GPS-Denied Operation:
```
EK3_GPS_TYPE = 3         # No GPS
EK3_FLOW_USE = 1         # Use optical flow
EK3_RNG_USE_HGT = 70     # Use rangefinder for height
```

#### High-Vibration Environment:
```
EK3_ACC_P_NSE = 0.5      # Higher accel noise
EK3_GYRO_P_NSE = 0.03    # Higher gyro noise
EK3_ABIAS_P_NSE = 0.005  # Higher accel bias noise
```

## Performance and Tuning

### Performance Monitoring:

#### Key Metrics:
1. **Innovation Magnitudes**: Should be small and consistent
2. **Innovation Test Ratios**: Should be < 1.0 for healthy operation
3. **State Variances**: Indicate estimation uncertainty
4. **Core Selection**: Frequency of primary core changes

#### Logging and Analysis:

ArduPilot provides extensive EKF logging:
- **XKF1-XKF5**: Core state estimates and innovations
- **XKQ**: Quaternion states
- **XKFS**: Filter status and health metrics
- **XKV**: Velocity estimates and innovations

### Tuning Guidelines:

#### Initial Setup:
1. **Verify Sensor Installation**: Proper orientation and calibration
2. **Check Parameter Defaults**: Use appropriate vehicle-specific defaults
3. **Monitor Initial Flight**: Review logs for any obvious issues
4. **Adjust Gradually**: Make small parameter changes and test

#### Common Tuning Scenarios:

##### High Innovation Magnitudes:
- **Cause**: Sensor noise, mounting issues, or parameter mismatch
- **Solution**: Increase measurement noise parameters or improve mounting

##### Frequent Core Switching:
- **Cause**: Inconsistent sensor performance or aggressive thresholds
- **Solution**: Adjust innovation gates or improve sensor quality

##### Poor GPS Performance:
- **Cause**: Multipath, interference, or poor antenna placement
- **Solution**: Improve GPS setup or increase GPS noise parameters

##### Magnetometer Issues:
- **Cause**: Magnetic interference or poor calibration
- **Solution**: Improve mounting, calibration, or switch to GPS heading

## Troubleshooting

### Common Issues:

#### 1. EKF Failsafe Triggered
**Symptoms**: "EKF Primary Changed" or "EKF Variance" warnings
**Causes**:
- Poor sensor data quality
- Excessive vibrations
- Magnetic interference
- GPS issues

**Solutions**:
- Check sensor mounting and calibration
- Improve vibration damping
- Move GPS/compass away from interference sources
- Adjust noise parameters

#### 2. Poor Position Hold
**Symptoms**: Vehicle drifts or oscillates in position modes
**Causes**:
- GPS accuracy issues
- Optical flow problems
- Wind estimation errors

**Solutions**:
- Improve GPS reception
- Calibrate optical flow sensor
- Check wind parameter tuning

#### 3. Altitude Errors
**Symptoms**: Unexpected altitude changes or oscillations
**Causes**:
- Barometer interference
- Ground effect on barometer
- Poor GPS vertical accuracy

**Solutions**:
- Improve barometer mounting
- Use rangefinder for low altitude
- Adjust height source priorities

#### 4. Heading Drift
**Symptoms**: Vehicle heading slowly changes over time
**Causes**:
- Poor compass calibration
- Magnetic interference
- GPS heading unavailable

**Solutions**:
- Perform thorough compass calibration
- Reduce magnetic interference
- Enable GPS heading fusion

### Diagnostic Tools:

#### Log Analysis:
1. **Plot Innovation Test Ratios**: Should be < 1.0
2. **Check State Variances**: Excessive growth indicates problems
3. **Monitor Core Health**: Frequent switching suggests issues
4. **Analyze Sensor Data**: Look for outliers or dropouts

#### Real-Time Monitoring:
1. **GCS Displays**: Monitor EKF status in ground control software
2. **MAVLink Messages**: EKF_STATUS_REPORT provides detailed health info
3. **Parameter Monitoring**: Watch key innovation and variance parameters

## Advanced Topics

### Multi-Core Operation:

EKF3 can run multiple cores with different configurations:
- **Core 0**: Primary GPS + compass configuration
- **Core 1**: Backup with different GPS or no compass
- **Core 2**: GPS-denied with optical flow
- **Core 3-5**: Additional configurations as needed

**Benefits**:
- Automatic failover during sensor failures
- Optimized performance for different flight phases
- Improved reliability in challenging environments

### External Navigation Integration:

Modern EKF implementations support external navigation sources:

#### Visual-Inertial Odometry (VIO):
- Intel T265 camera integration
- Position and orientation from visual tracking
- Complements or replaces GPS in indoor/GPS-denied environments

#### Motion Capture Systems:
- VICON, OptiTrack integration
- High-precision position and orientation
- Ideal for research and indoor applications

#### Wheel Odometry:
- Ground vehicle wheel encoder integration
- Velocity estimation for rovers
- Reduces GPS dependency for ground navigation

### Lane Switching Algorithm:

EKF3 implements sophisticated core selection:

```cpp
// Simplified lane switching logic
if (innovation_test_ratio > threshold) {
    // Current core failing, check alternatives
    for (core : available_cores) {
        if (core.innovation_test_ratio < current.innovation_test_ratio * margin) {
            // Switch to better performing core
            switch_primary_core(core);
            break;
        }
    }
}
```

### GPS-Denied Navigation:

For missions without GPS, EKF3 supports:
1. **Optical Flow**: Visual velocity estimation
2. **Rangefinder**: Terrain-relative altitude
3. **External Navigation**: Camera-based positioning
4. **Dead Reckoning**: Pure inertial navigation (limited duration)

**Configuration Example**:
```
EK3_GPS_TYPE = 3          # No GPS
EK3_FLOW_USE = 1          # Enable optical flow
EK3_RNG_USE_HGT = 70      # Rangefinder for height
EK3_RNG_USE_SPD = 2.0     # Max speed for rangefinder use
```

### Sensor Affinity:

EKF3 supports sensor affinity to bind cores to specific sensors:
```
EK3_AFFINITY: Bitmask controlling sensor assignment
- Bit 0: GPS affinity
- Bit 1: Barometer affinity  
- Bit 2: Compass affinity
- Bit 3: Airspeed affinity
```

This ensures consistent sensor usage across cores and prevents interference.

### Future Developments:

The EKF system continues to evolve with:
1. **Machine Learning Integration**: AI-assisted sensor fusion
2. **Advanced Fault Detection**: Predictive maintenance capabilities
3. **Improved GPS-Denied Navigation**: Enhanced visual-inertial algorithms
4. **Multi-Vehicle Cooperation**: Distributed state estimation
5. **Edge Computing**: Optimized algorithms for embedded systems

---

*This documentation provides a comprehensive overview of EKF implementation in ArduPilot. For the most current information, always refer to the official ArduPilot documentation and source code.*