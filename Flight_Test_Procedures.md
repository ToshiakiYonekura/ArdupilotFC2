# Flight Test Procedures for System Identification

## Table of Contents
1. [Test Planning and Preparation](#test-planning)
2. [Test Environment Setup](#environment-setup)
3. [Specific Test Procedures](#test-procedures)
4. [Data Quality Assurance](#data-quality)
5. [Emergency Procedures](#emergency-procedures)
6. [Post-Flight Analysis](#post-flight-analysis)

## Test Planning and Preparation {#test-planning}

### Flight Test Matrix

```python
class FlightTestMatrix:
    def __init__(self):
        self.test_matrix = {
            'basic_identification': {
                'hover_trim': {
                    'objective': 'Determine hover throttle and trim settings',
                    'duration': 300,  # seconds
                    'conditions': ['calm_wind', 'stable_gps'],
                    'data_required': ['throttle_position', 'attitude', 'altitude']
                },
                'control_response': {
                    'objective': 'Measure basic control response',
                    'duration': 600,
                    'conditions': ['hover_stable'],
                    'data_required': ['control_input', 'angular_rate', 'attitude']
                }
            },
            'frequency_domain': {
                'roll_axis_sweep': {
                    'objective': 'Identify roll axis dynamics',
                    'frequency_range': (1.0, 25.0),
                    'magnitude': 15,
                    'duration': 25,
                    'conditions': ['althold_mode', 'good_gps']
                },
                'pitch_axis_sweep': {
                    'objective': 'Identify pitch axis dynamics', 
                    'frequency_range': (1.0, 25.0),
                    'magnitude': 15,
                    'duration': 25,
                    'conditions': ['althold_mode', 'good_gps']
                },
                'yaw_axis_sweep': {
                    'objective': 'Identify yaw axis dynamics',
                    'frequency_range': (0.5, 15.0),
                    'magnitude': 10,
                    'duration': 30,
                    'conditions': ['althold_mode', 'good_gps']
                }
            },
            'time_domain': {
                'step_response': {
                    'objective': 'Measure transient response characteristics',
                    'inputs': ['roll_step', 'pitch_step', 'yaw_step', 'throttle_step'],
                    'step_size': [10, 10, 15, 5],  # degrees or %
                    'duration': 120,
                    'conditions': ['manual_control', 'experienced_pilot']
                },
                'doublet_response': {
                    'objective': 'Identify control effectiveness',
                    'inputs': ['3211_doublet'],
                    'duration': 60,
                    'conditions': ['manual_control', 'experienced_pilot']
                }
            },
            'performance_envelope': {
                'max_rate_test': {
                    'objective': 'Determine maximum angular rates',
                    'duration': 180,
                    'conditions': ['experienced_pilot', 'open_airspace'],
                    'safety_limits': {'roll_rate': 360, 'pitch_rate': 360, 'yaw_rate': 180}
                },
                'control_authority': {
                    'objective': 'Measure control authority limits',
                    'duration': 240,
                    'conditions': ['various_flight_speeds', 'different_altitudes']
                }
            }
        }
        
    def generate_test_plan(self, vehicle_config, pilot_experience, weather_conditions):
        """Generate customized test plan based on conditions"""
        
        plan = []
        
        # Always start with basic identification
        if self.check_conditions(['calm_wind', 'stable_gps'], weather_conditions):
            plan.extend(list(self.test_matrix['basic_identification'].keys()))
            
        # Add frequency domain tests if conditions allow
        if self.check_conditions(['althold_mode', 'good_gps'], weather_conditions):
            plan.extend(list(self.test_matrix['frequency_domain'].keys()))
            
        # Add time domain tests if pilot is experienced
        if pilot_experience >= 'intermediate':
            plan.extend(list(self.test_matrix['time_domain'].keys()))
            
        # Add performance tests only for experienced pilots
        if pilot_experience == 'expert':
            plan.extend(list(self.test_matrix['performance_envelope'].keys()))
            
        return plan
        
    def check_conditions(self, required_conditions, current_conditions):
        """Check if current conditions meet requirements"""
        return all(cond in current_conditions for cond in required_conditions)

# Usage example
test_matrix = FlightTestMatrix()
weather = ['calm_wind', 'stable_gps', 'good_gps', 'althold_mode']
pilot_level = 'intermediate'
vehicle = {'type': 'hexacopter', 'mass': 2.5}

test_plan = test_matrix.generate_test_plan(vehicle, pilot_level, weather)
print("Recommended test sequence:", test_plan)
```

### Pre-Flight Planning Checklist

```python
class PreFlightPlanning:
    def __init__(self):
        self.planning_checklist = {
            'flight_area': {
                'minimum_size': '50m x 50m',
                'obstacles': 'None within 100m radius',
                'emergency_landing': 'Multiple options identified',
                'airspace': 'Authorization obtained if required',
                'ground_conditions': 'Level takeoff/landing area'
            },
            'weather_limits': {
                'wind_speed': '<5 m/s sustained, <8 m/s gusts',
                'visibility': '>3 km',
                'precipitation': 'None',
                'temperature': '>0°C, <40°C',
                'density_altitude': '<3000m'
            },
            'vehicle_requirements': {
                'battery_capacity': '>80% charged',
                'backup_battery': 'Fully charged',
                'propellers': 'Balanced and undamaged',
                'control_surfaces': 'Full range of motion',
                'sensors': 'All calibrated within 7 days'
            },
            'crew_requirements': {
                'pilot': 'Current and proficient',
                'safety_observer': 'Designated and briefed',
                'test_director': 'Familiar with test procedures',
                'emergency_contact': 'Local authorities notified'
            },
            'equipment_requirements': {
                'ground_station': 'Laptop with telemetry',
                'backup_radio': 'Manual control capability',
                'first_aid': 'Kit accessible',
                'fire_extinguisher': 'CO2 type for electrical fires',
                'data_storage': 'Multiple backup devices'
            }
        }
        
    def conduct_planning_review(self):
        """Conduct comprehensive pre-flight planning review"""
        
        print("Flight Test Planning Review")
        print("=" * 50)
        
        all_items_checked = True
        
        for category, items in self.planning_checklist.items():
            print(f"\n{category.upper().replace('_', ' ')}:")
            for item, requirement in items.items():
                status = input(f"  {item.replace('_', ' ')}: {requirement} - OK? (y/n): ")
                if status.lower() != 'y':
                    all_items_checked = False
                    print(f"    ✗ {item} not satisfied")
                else:
                    print(f"    ✓ {item} verified")
                    
        print("=" * 50)
        if all_items_checked:
            print("✓ All planning items verified - CLEARED FOR FLIGHT TESTING")
        else:
            print("✗ Planning items incomplete - DO NOT PROCEED")
            
        return all_items_checked
        
    def calculate_flight_time_budget(self, battery_capacity_mah, average_current_a):
        """Calculate available flight time for testing"""
        
        # Conservative calculation with safety margins
        usable_capacity = battery_capacity_mah * 0.8  # 80% usable
        safety_margin = 0.7  # 30% reserve for landing
        
        theoretical_time = (usable_capacity / 1000) / average_current_a * 60  # minutes
        safe_flight_time = theoretical_time * safety_margin
        
        print(f"Flight Time Budget:")
        print(f"  Battery capacity: {battery_capacity_mah} mAh")
        print(f"  Average current: {average_current_a} A")
        print(f"  Theoretical time: {theoretical_time:.1f} min")
        print(f"  Safe flight time: {safe_flight_time:.1f} min")
        
        return safe_flight_time

# Usage
planner = PreFlightPlanning()
# planner.conduct_planning_review()
flight_time = planner.calculate_flight_time_budget(10000, 25)  # 10Ah battery, 25A average
```

## Test Environment Setup {#environment-setup}

### Ground Station Configuration

```python
class GroundStationSetup:
    def __init__(self):
        self.required_software = {
            'mission_planner': 'Latest stable version',
            'mavproxy': 'For command line interface', 
            'pymavlink': 'For custom scripts',
            'matlab_simulink': 'For real-time analysis',
            'python_environment': 'With numpy, scipy, matplotlib'
        }
        
        self.telemetry_config = {
            'primary_link': {
                'type': '915MHz radio',
                'baud_rate': 57600,
                'range': '>1km',
                'latency': '<50ms'
            },
            'backup_link': {
                'type': 'WiFi or cellular',
                'purpose': 'Emergency control',
                'range': 'As available'
            }
        }
        
    def setup_logging_parameters(self):
        """Configure logging for system identification"""
        
        logging_config = {
            # High-rate IMU logging
            'LOG_BITMASK': 176126,  # All required messages
            'INS_LOG_BAT_MASK': 3,  # Log first 2 IMUs
            'INS_LOG_BAT_OPT': 0,   # Standard logging
            
            # System ID specific logging
            'SID_LOG': 1,           # Enable SID logging
            
            # High-rate attitude logging
            'LOG_FILE_BUFSIZE': 64, # Larger buffer for high-rate logging
            'LOG_DISARMED': 0,      # Don't log when disarmed
            'LOG_REPLAY': 1,        # Enable log replay capability
            
            # Sensor data rates
            'AHRS_EKF_TYPE': 3,     # Use EKF3
            'EK3_ENABLE': 1,        # Enable EKF3
            'INS_GYRO_FILTER': 20,  # Gyro filter frequency
            'INS_ACCEL_FILTER': 20, # Accel filter frequency
        }
        
        print("System ID Logging Configuration:")
        for param, value in logging_config.items():
            print(f"  {param}: {value}")
            
        return logging_config
        
    def setup_real_time_monitoring(self):
        """Setup real-time data monitoring"""
        
        monitoring_script = """
        # Real-time monitoring setup
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        import numpy as np
        from collections import deque
        
        class RealTimeMonitor:
            def __init__(self, connection):
                self.connection = connection
                self.data_buffer = {
                    'time': deque(maxlen=1000),
                    'roll': deque(maxlen=1000),
                    'pitch': deque(maxlen=1000), 
                    'yaw': deque(maxlen=1000),
                    'roll_rate': deque(maxlen=1000),
                    'pitch_rate': deque(maxlen=1000),
                    'yaw_rate': deque(maxlen=1000),
                    'frequency': deque(maxlen=1000),
                    'magnitude': deque(maxlen=1000)
                }
                
                # Setup plots
                self.fig, self.axes = plt.subplots(3, 2, figsize=(15, 10))
                self.setup_plots()
                
            def setup_plots(self):
                # Attitude plots
                self.axes[0,0].set_title('Attitude (degrees)')
                self.axes[0,0].set_ylabel('Angle (deg)')
                
                # Angular rate plots  
                self.axes[0,1].set_title('Angular Rates (deg/s)')
                self.axes[0,1].set_ylabel('Rate (deg/s)')
                
                # System ID progress
                self.axes[1,0].set_title('SID Frequency Sweep')
                self.axes[1,0].set_ylabel('Frequency (Hz)')
                
                # System ID magnitude
                self.axes[1,1].set_title('SID Input Magnitude')
                self.axes[1,1].set_ylabel('Magnitude (%)')
                
                # Live frequency response
                self.axes[2,0].set_title('Live Bode Plot - Magnitude')
                self.axes[2,0].set_ylabel('Magnitude (dB)')
                self.axes[2,0].set_xlabel('Frequency (Hz)')
                
                # Live frequency response - phase
                self.axes[2,1].set_title('Live Bode Plot - Phase')
                self.axes[2,1].set_ylabel('Phase (deg)')
                self.axes[2,1].set_xlabel('Frequency (Hz)')
        """
        
        return monitoring_script

    def verify_communication_links(self):
        """Verify all communication links are working"""
        
        tests = [
            'Primary telemetry link latency test',
            'Backup communication verification',
            'Command acknowledgment test',
            'Parameter read/write test',
            'Emergency stop command test'
        ]
        
        print("Communication Link Verification:")
        for test in tests:
            result = input(f"  {test} - PASS? (y/n): ")
            if result.lower() == 'y':
                print(f"    ✓ {test}")
            else:
                print(f"    ✗ {test} - INVESTIGATE BEFORE PROCEEDING")
                
        return True  # Simplified for example
```

### Test Site Survey

```python
class TestSiteSurvey:
    def __init__(self):
        self.site_requirements = {
            'dimensions': {
                'minimum_area': '50m x 50m',
                'preferred_area': '100m x 100m',
                'height_clearance': '120m AGL minimum'
            },
            'obstacles': {
                'trees': 'Map all trees >10m height within 200m',
                'buildings': 'Map all structures within 500m',
                'power_lines': 'Identify all power lines within 1km',
                'roads': 'Note traffic patterns and busy periods'
            },
            'environmental': {
                'wind_patterns': 'Observe typical wind direction/speed',
                'thermal_activity': 'Note thermal generation areas',
                'electromagnetic': 'Check for interference sources',
                'wildlife': 'Identify bird activity patterns'
            }
        }
        
    def conduct_site_survey(self):
        """Conduct comprehensive test site survey"""
        
        survey_data = {}
        
        print("Test Site Survey")
        print("=" * 30)
        
        # GPS coordinates
        lat = float(input("Site latitude (decimal degrees): "))
        lon = float(input("Site longitude (decimal degrees): "))
        alt = float(input("Site elevation (meters MSL): "))
        
        survey_data['coordinates'] = {'lat': lat, 'lon': lon, 'alt': alt}
        
        # Dimensions measurement
        print("\nSite Dimensions:")
        length = float(input("  Available length (m): "))
        width = float(input("  Available width (m): "))
        
        survey_data['dimensions'] = {'length': length, 'width': width}
        
        if length < 50 or width < 50:
            print("  ⚠️  WARNING: Site may be too small for safe system ID testing")
            
        # Obstacle mapping
        print("\nObstacle Survey:")
        obstacles = []
        
        while True:
            obstacle_type = input("  Obstacle type (tree/building/power/done): ")
            if obstacle_type.lower() == 'done':
                break
                
            distance = float(input(f"    Distance to {obstacle_type} (m): "))
            height = float(input(f"    Height of {obstacle_type} (m): "))
            bearing = float(input(f"    Bearing to {obstacle_type} (degrees): "))
            
            obstacles.append({
                'type': obstacle_type,
                'distance': distance,
                'height': height,
                'bearing': bearing
            })
            
        survey_data['obstacles'] = obstacles
        
        # Wind assessment
        print("\nWind Assessment:")
        typical_wind_dir = float(input("  Typical wind direction (degrees): "))
        typical_wind_speed = float(input("  Typical wind speed (m/s): "))
        
        survey_data['wind'] = {
            'direction': typical_wind_dir,
            'speed': typical_wind_speed
        }
        
        # Generate safety assessment
        safety_score = self.assess_site_safety(survey_data)
        
        print(f"\nSite Safety Score: {safety_score}/100")
        if safety_score >= 80:
            print("✓ Site approved for system ID testing")
        elif safety_score >= 60:
            print("⚠️  Site marginal - proceed with extra caution")
        else:
            print("✗ Site not recommended for system ID testing")
            
        return survey_data, safety_score
        
    def assess_site_safety(self, survey_data):
        """Assess site safety based on survey data"""
        
        score = 100
        
        # Deduct points for small area
        area = survey_data['dimensions']['length'] * survey_data['dimensions']['width']
        if area < 2500:  # 50m x 50m
            score -= 30
        elif area < 10000:  # 100m x 100m
            score -= 10
            
        # Deduct points for nearby obstacles
        for obstacle in survey_data['obstacles']:
            if obstacle['distance'] < 50:
                score -= 20
            elif obstacle['distance'] < 100:
                score -= 10
                
        # Deduct points for high wind
        if survey_data['wind']['speed'] > 8:
            score -= 30
        elif survey_data['wind']['speed'] > 5:
            score -= 15
            
        return max(0, score)
```

## Specific Test Procedures {#test-procedures}

### Hover Trim and Basic Stability Test

```python
class HoverTrimTest:
    def __init__(self, vehicle_connection):
        self.vehicle = vehicle_connection
        self.test_data = {
            'hover_throttle': [],
            'trim_roll': [],
            'trim_pitch': [],
            'trim_yaw': [],
            'altitude_hold_accuracy': [],
            'position_hold_accuracy': []
        }
        
    def execute_hover_trim_test(self):
        """Execute comprehensive hover trim identification"""
        
        print("Hover Trim and Basic Stability Test")
        print("=" * 40)
        
        # Phase 1: Manual hover trim
        print("\nPhase 1: Manual Hover Trim")
        manual_trim = self.manual_hover_trim()
        
        # Phase 2: Altitude hold accuracy
        print("\nPhase 2: Altitude Hold Test")
        alt_hold_data = self.altitude_hold_test()
        
        # Phase 3: Position hold accuracy  
        print("\nPhase 3: Position Hold Test")
        pos_hold_data = self.position_hold_test()
        
        # Phase 4: Basic stability margins
        print("\nPhase 4: Stability Margin Test")
        stability_data = self.stability_margin_test()
        
        # Compile results
        results = {
            'manual_trim': manual_trim,
            'altitude_hold': alt_hold_data,
            'position_hold': pos_hold_data,
            'stability': stability_data
        }
        
        self.analyze_hover_trim_results(results)
        return results
        
    def manual_hover_trim(self):
        """Determine manual hover trim settings"""
        
        print("Instructions for pilot:")
        print("1. Take off and establish stable hover at 5m AGL")
        print("2. Maintain position using minimal stick inputs")
        print("3. Note required stick positions for level flight")
        print("4. Test will record for 60 seconds")
        
        input("Press Enter when ready to start recording...")
        
        # Record 60 seconds of hover data
        start_time = time.time()
        hover_data = {
            'throttle': [],
            'roll_trim': [],
            'pitch_trim': [],
            'yaw_trim': [],
            'altitude': [],
            'position_n': [],
            'position_e': []
        }
        
        print("Recording hover trim data...")
        while time.time() - start_time < 60:
            # Collect data (this would interface with actual telemetry)
            msg = self.vehicle.recv_match(type=['RC_CHANNELS', 'GLOBAL_POSITION_INT'], blocking=False)
            
            if msg and msg.get_type() == 'RC_CHANNELS':
                hover_data['throttle'].append(msg.chan3_raw)
                hover_data['roll_trim'].append(msg.chan1_raw - 1500)  # Trim offset
                hover_data['pitch_trim'].append(msg.chan2_raw - 1500)
                hover_data['yaw_trim'].append(msg.chan4_raw - 1500)
                
            if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
                hover_data['altitude'].append(msg.relative_alt / 1000.0)
                hover_data['position_n'].append(msg.lat / 1e7)
                hover_data['position_e'].append(msg.lon / 1e7)
                
            time.sleep(0.1)  # 10 Hz data collection
            
        # Calculate trim values
        trim_results = {
            'hover_throttle': np.mean(hover_data['throttle']),
            'hover_throttle_std': np.std(hover_data['throttle']),
            'roll_trim': np.mean(hover_data['roll_trim']),
            'pitch_trim': np.mean(hover_data['pitch_trim']),
            'yaw_trim': np.mean(hover_data['yaw_trim']),
            'altitude_stability': np.std(hover_data['altitude']),
            'position_stability_n': np.std(hover_data['position_n']),
            'position_stability_e': np.std(hover_data['position_e'])
        }
        
        print(f"Hover trim results:")
        print(f"  Hover throttle: {trim_results['hover_throttle']:.0f} PWM")
        print(f"  Roll trim: {trim_results['roll_trim']:.0f} PWM")
        print(f"  Pitch trim: {trim_results['pitch_trim']:.0f} PWM")
        print(f"  Altitude std: {trim_results['altitude_stability']:.2f} m")
        
        return trim_results
        
    def altitude_hold_test(self):
        """Test altitude hold accuracy and response"""
        
        print("Instructions:")
        print("1. Switch to ALTHOLD mode")
        print("2. Climb to 10m, then 15m, then 20m")
        print("3. Hold each altitude for 30 seconds")
        print("4. Return to 5m hover")
        
        input("Press Enter when ready to start altitude hold test...")
        
        altitude_targets = [10, 15, 20, 5]  # meters
        alt_hold_data = []
        
        for target_alt in altitude_targets:
            print(f"Climb to {target_alt}m and hold...")
            input("Press Enter when at target altitude...")
            
            # Record 30 seconds of altitude hold data
            start_time = time.time()
            alt_data = []
            
            while time.time() - start_time < 30:
                msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg:
                    current_alt = msg.relative_alt / 1000.0
                    alt_data.append(current_alt)
                time.sleep(0.1)
                
            # Calculate statistics
            alt_stats = {
                'target': target_alt,
                'mean': np.mean(alt_data),
                'std': np.std(alt_data),
                'error': np.mean(alt_data) - target_alt,
                'max_deviation': np.max(np.abs(np.array(alt_data) - target_alt))
            }
            
            alt_hold_data.append(alt_stats)
            print(f"  Mean altitude: {alt_stats['mean']:.2f}m")
            print(f"  Error: {alt_stats['error']:.2f}m")
            print(f"  Standard deviation: {alt_stats['std']:.2f}m")
            
        return alt_hold_data
        
    def position_hold_test(self):
        """Test position hold accuracy in LOITER mode"""
        
        print("Instructions:")
        print("1. Switch to LOITER mode")
        print("2. Release all controls")
        print("3. Test will record position hold for 2 minutes")
        print("4. Note any wind disturbances")
        
        input("Press Enter when ready to start position hold test...")
        
        # Record 2 minutes of position hold data
        start_time = time.time()
        pos_data = {'lat': [], 'lon': [], 'alt': []}
        
        print("Recording position hold data for 2 minutes...")
        while time.time() - start_time < 120:
            msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                pos_data['lat'].append(msg.lat / 1e7)
                pos_data['lon'].append(msg.lon / 1e7) 
                pos_data['alt'].append(msg.relative_alt / 1000.0)
            time.sleep(0.2)  # 5 Hz for position
            
        # Calculate position hold statistics
        # Convert to local coordinates for easier interpretation
        lat_array = np.array(pos_data['lat'])
        lon_array = np.array(pos_data['lon'])
        
        # Simple conversion to meters (approximate)
        lat_center = np.mean(lat_array)
        lon_center = np.mean(lon_array)
        
        north_pos = (lat_array - lat_center) * 111319.9  # deg to meters
        east_pos = (lon_array - lon_center) * 111319.9 * np.cos(np.radians(lat_center))
        
        pos_hold_stats = {
            'north_std': np.std(north_pos),
            'east_std': np.std(east_pos),
            'alt_std': np.std(pos_data['alt']),
            'max_deviation_horizontal': np.max(np.sqrt(north_pos**2 + east_pos**2)),
            'max_deviation_vertical': np.max(np.abs(pos_data['alt'] - np.mean(pos_data['alt']))),
            'circular_error_90': np.percentile(np.sqrt(north_pos**2 + east_pos**2), 90)
        }
        
        print(f"Position hold results:")
        print(f"  Horizontal std: {pos_hold_stats['north_std']:.2f}m N, {pos_hold_stats['east_std']:.2f}m E")
        print(f"  Vertical std: {pos_hold_stats['alt_std']:.2f}m")
        print(f"  90% error circle: {pos_hold_stats['circular_error_90']:.2f}m")
        
        return pos_hold_stats

# Usage example
# hover_test = HoverTrimTest(vehicle_connection)
# results = hover_test.execute_hover_trim_test()
```

### Step Response Test Procedures

```python
class StepResponseTest:
    def __init__(self, vehicle_connection):
        self.vehicle = vehicle_connection
        self.test_sequence = {
            'roll_steps': [5, 10, 15, -5, -10, -15],  # degrees
            'pitch_steps': [5, 10, 15, -5, -10, -15],
            'yaw_steps': [10, 20, 30, -10, -20, -30],
            'throttle_steps': [5, 10, -5, -10]  # percent
        }
        
    def execute_step_response_test(self):
        """Execute comprehensive step response testing"""
        
        print("Step Response Test Procedure")
        print("=" * 35)
        
        # Pre-test briefing
        self.pilot_briefing()
        
        results = {}
        
        # Test each axis
        for axis in ['roll', 'pitch', 'yaw', 'throttle']:
            print(f"\n--- {axis.upper()} AXIS STEP RESPONSE ---")
            
            axis_results = self.test_axis_step_response(axis)
            results[axis] = axis_results
            
            # Rest period between axes
            if axis != 'throttle':  # No rest after last test
                print("Rest period - return to stable hover for 30 seconds...")
                time.sleep(30)
                
        # Analyze all results
        self.analyze_step_response_results(results)
        return results
        
    def pilot_briefing(self):
        """Brief pilot on step response procedures"""
        
        briefing = """
        STEP RESPONSE TEST BRIEFING:
        
        1. Establish stable hover at 10m AGL
        2. When commanded, apply SHARP step input to specified axis
        3. Hold input for 2 seconds, then QUICKLY return to neutral
        4. Hold neutral for 8 seconds (allow response to settle)
        5. Maintain altitude and position during test
        6. Call "STEP" when applying input, "NEUTRAL" when releasing
        
        IMPORTANT SAFETY NOTES:
        - Do not exceed specified input magnitudes
        - Be ready to recover if vehicle becomes unstable
        - Abort test if position drifts beyond 20m from start point
        - Maximum test altitude: 15m AGL
        """
        
        print(briefing)
        pilot_ready = input("Pilot ready to proceed? (y/n): ")
        
        if pilot_ready.lower() != 'y':
            raise Exception("Pilot not ready - aborting test")
            
    def test_axis_step_response(self, axis):
        """Test step response for specific axis"""
        
        steps = self.test_sequence[f'{axis}_steps']
        axis_results = []
        
        for step_magnitude in steps:
            print(f"\nPreparing {step_magnitude}° {axis} step...")
            
            # Pre-step stabilization
            input("Establish stable hover, then press Enter...")
            
            print("Starting data recording...")
            print(f"Apply {step_magnitude}° {axis} step in 3... 2... 1... NOW!")
            
            # Record step response
            step_data = self.record_step_response(axis, step_magnitude)
            
            # Analyze this step
            step_analysis = self.analyze_single_step(step_data, axis, step_magnitude)
            axis_results.append(step_analysis)
            
            print(f"Step completed. Rise time: {step_analysis['rise_time']:.2f}s, "
                  f"Overshoot: {step_analysis['overshoot']:.1f}%")
                  
        return axis_results
        
    def record_step_response(self, axis, step_magnitude):
        """Record data during step response"""
        
        # Record for 10 seconds total
        start_time = time.time()
        step_data = {
            'time': [],
            'input': [],
            'attitude_roll': [],
            'attitude_pitch': [],
            'attitude_yaw': [],
            'rate_roll': [],
            'rate_pitch': [],
            'rate_yaw': [],
            'throttle': [],
            'altitude': []
        }
        
        while time.time() - start_time < 10.0:
            current_time = time.time() - start_time
            
            # Get attitude data
            attitude_msg = self.vehicle.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                step_data['time'].append(current_time)
                step_data['attitude_roll'].append(math.degrees(attitude_msg.roll))
                step_data['attitude_pitch'].append(math.degrees(attitude_msg.pitch))
                step_data['attitude_yaw'].append(math.degrees(attitude_msg.yaw))
                step_data['rate_roll'].append(math.degrees(attitude_msg.rollspeed))
                step_data['rate_pitch'].append(math.degrees(attitude_msg.pitchspeed))
                step_data['rate_yaw'].append(math.degrees(attitude_msg.yawspeed))
                
            # Get RC input data
            rc_msg = self.vehicle.recv_match(type='RC_CHANNELS', blocking=False)
            if rc_msg:
                if axis == 'roll':
                    step_data['input'].append((rc_msg.chan1_raw - 1500) / 500 * 45)  # Convert to degrees
                elif axis == 'pitch':
                    step_data['input'].append((rc_msg.chan2_raw - 1500) / 500 * 45)
                elif axis == 'yaw':
                    step_data['input'].append((rc_msg.chan4_raw - 1500) / 500 * 45)
                elif axis == 'throttle':
                    step_data['input'].append((rc_msg.chan3_raw - 1000) / 1000 * 100)  # Percent
                    
            # Get altitude data
            alt_msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if alt_msg:
                step_data['altitude'].append(alt_msg.relative_alt / 1000.0)
                
            time.sleep(0.02)  # 50 Hz sampling
            
        return step_data
        
    def analyze_single_step(self, step_data, axis, step_magnitude):
        """Analyze single step response"""
        
        # Convert to numpy arrays
        time_array = np.array(step_data['time'])
        input_array = np.array(step_data['input'])
        
        if axis == 'roll':
            output_array = np.array(step_data['attitude_roll'])
            rate_array = np.array(step_data['rate_roll'])
        elif axis == 'pitch':
            output_array = np.array(step_data['attitude_pitch'])
            rate_array = np.array(step_data['rate_pitch'])
        elif axis == 'yaw':
            output_array = np.array(step_data['attitude_yaw'])
            rate_array = np.array(step_data['rate_yaw'])
        elif axis == 'throttle':
            output_array = np.array(step_data['altitude'])
            rate_array = np.gradient(output_array) / np.gradient(time_array)  # Climb rate
            
        # Find step start (when input changes significantly)
        step_start_idx = np.where(np.abs(np.gradient(input_array)) > abs(step_magnitude) * 0.5)[0]
        if len(step_start_idx) > 0:
            step_start_idx = step_start_idx[0]
        else:
            step_start_idx = 10  # Default if not found
            
        # Extract response portion
        response_time = time_array[step_start_idx:]
        response_output = output_array[step_start_idx:]
        response_rate = rate_array[step_start_idx:]
        
        # Reset time to start from 0
        response_time = response_time - response_time[0]
        
        # Calculate step response parameters
        initial_value = response_output[0]
        final_value = np.mean(response_output[-20:])  # Average of last 20 samples
        step_size = final_value - initial_value
        
        # Rise time (10% to 90%)
        rise_10_percent = initial_value + 0.1 * step_size
        rise_90_percent = initial_value + 0.9 * step_size
        
        rise_10_idx = np.where(response_output >= rise_10_percent)[0]
        rise_90_idx = np.where(response_output >= rise_90_percent)[0]
        
        if len(rise_10_idx) > 0 and len(rise_90_idx) > 0:
            rise_time = response_time[rise_90_idx[0]] - response_time[rise_10_idx[0]]
        else:
            rise_time = np.nan
            
        # Overshoot
        peak_value = np.max(response_output) if step_size > 0 else np.min(response_output)
        if abs(step_size) > 0.1:  # Avoid division by zero
            overshoot = abs(peak_value - final_value) / abs(step_size) * 100
        else:
            overshoot = 0
            
        # Settling time (within 2% of final value)
        settling_band = 0.02 * abs(step_size)
        settled_indices = np.where(np.abs(response_output - final_value) <= settling_band)[0]
        
        if len(settled_indices) > 10:  # Need sustained settling
            settling_time = response_time[settled_indices[10]]  # When it first settles
        else:
            settling_time = response_time[-1]  # Didn't settle in test time
            
        # Steady-state error
        steady_state_error = abs(final_value - (initial_value + step_magnitude))
        
        analysis = {
            'axis': axis,
            'step_magnitude': step_magnitude,
            'rise_time': rise_time,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'steady_state_error': steady_state_error,
            'step_size_actual': step_size,
            'peak_rate': np.max(np.abs(response_rate)),
            'time_to_peak': response_time[np.argmax(np.abs(response_output - initial_value))],
            'raw_data': {
                'time': response_time,
                'input': input_array[step_start_idx:],
                'output': response_output,
                'rate': response_rate
            }
        }
        
        return analysis
```

This comprehensive flight test procedure guide provides:

1. **Complete test planning** with customized test matrices
2. **Professional safety procedures** and site surveys
3. **Detailed test execution** for hover trim and step responses
4. **Real-time monitoring** and data collection
5. **Automated analysis** of test results

The procedures are designed for safe, systematic identification of vehicle dynamics with proper safety margins and professional flight test practices.

Would you like me to continue with the Data Analysis Techniques and Complete Implementation Workflow guides?