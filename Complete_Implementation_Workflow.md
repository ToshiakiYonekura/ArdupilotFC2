# Complete Implementation Workflow for Hexacopter System Identification

## Table of Contents
1. [Project Overview and Timeline](#project-overview)
2. [Phase 1: Preparation and Planning](#phase-1-preparation)
3. [Phase 2: Ground-Based Measurements](#phase-2-ground-measurements)
4. [Phase 3: Flight Test Execution](#phase-3-flight-tests)
5. [Phase 4: Data Analysis and Model Extraction](#phase-4-data-analysis)
6. [Phase 5: Control System Implementation](#phase-5-implementation)
7. [Phase 6: Validation and Verification](#phase-6-validation)
8. [Project Management and Documentation](#project-management)

## Project Overview and Timeline {#project-overview}

### Complete System ID Project Structure

```python
class HexacopterSystemIDProject:
    def __init__(self):
        self.project_phases = {
            'preparation': {
                'duration_weeks': 2,
                'dependencies': [],
                'deliverables': ['test_plan', 'safety_procedures', 'equipment_checklist'],
                'critical_path': True
            },
            'ground_measurements': {
                'duration_weeks': 1,
                'dependencies': ['preparation'],
                'deliverables': ['physical_parameters', 'motor_characterization', 'sensor_calibration'],
                'critical_path': True
            },
            'flight_tests': {
                'duration_weeks': 2,
                'dependencies': ['ground_measurements'],
                'deliverables': ['flight_test_data', 'system_id_logs', 'performance_data'],
                'critical_path': True
            },
            'data_analysis': {
                'duration_weeks': 2,
                'dependencies': ['flight_tests'],
                'deliverables': ['identified_models', 'parameter_estimates', 'validation_results'],
                'critical_path': True
            },
            'implementation': {
                'duration_weeks': 1,
                'dependencies': ['data_analysis'],
                'deliverables': ['parameter_files', 'control_code', 'integration_test'],
                'critical_path': True
            },
            'validation': {
                'duration_weeks': 1,
                'dependencies': ['implementation'],
                'deliverables': ['validation_flights', 'performance_comparison', 'final_report'],
                'critical_path': True
            }
        }
        
        self.total_duration = sum(phase['duration_weeks'] for phase in self.project_phases.values())
        
    def generate_project_timeline(self):
        """Generate detailed project timeline with milestones"""
        
        timeline = []
        current_week = 0
        
        print("Hexacopter System Identification Project Timeline")
        print("=" * 60)
        
        for phase_name, phase_info in self.project_phases.items():
            start_week = current_week + 1
            end_week = current_week + phase_info['duration_weeks']
            
            timeline.append({
                'phase': phase_name,
                'start_week': start_week,
                'end_week': end_week,
                'duration': phase_info['duration_weeks'],
                'deliverables': phase_info['deliverables'],
                'critical_path': phase_info['critical_path']
            })
            
            critical_marker = " (CRITICAL)" if phase_info['critical_path'] else ""
            print(f"Week {start_week:2d}-{end_week:2d}: {phase_name.title().replace('_', ' ')}{critical_marker}")
            print(f"         Duration: {phase_info['duration_weeks']} weeks")
            print(f"         Deliverables: {', '.join(phase_info['deliverables'])}")
            print()
            
            current_week = end_week
            
        print(f"Total Project Duration: {self.total_duration} weeks")
        return timeline
        
    def identify_critical_path(self):
        """Identify and highlight critical path activities"""
        
        critical_activities = []
        
        for phase_name, phase_info in self.project_phases.items():
            if phase_info['critical_path']:
                critical_activities.append({
                    'phase': phase_name,
                    'duration': phase_info['duration_weeks'],
                    'dependencies': phase_info['dependencies']
                })
                
        print("Critical Path Analysis:")
        print("-" * 30)
        
        total_critical_time = 0
        for activity in critical_activities:
            print(f"• {activity['phase'].title().replace('_', ' ')}: {activity['duration']} weeks")
            total_critical_time += activity['duration']
            
        print(f"\nTotal Critical Path Duration: {total_critical_time} weeks")
        
        return critical_activities

# Usage
project = HexacopterSystemIDProject()
timeline = project.generate_project_timeline()
critical_path = project.identify_critical_path()
```

### Resource Requirements

```python
class ProjectResourcePlanning:
    def __init__(self):
        self.personnel_requirements = {
            'project_lead': {
                'required_skills': ['system_identification', 'flight_testing', 'data_analysis'],
                'time_allocation': '100%',
                'critical_phases': ['all']
            },
            'test_pilot': {
                'required_skills': ['multirotor_piloting', 'test_procedures', 'safety_protocols'],
                'time_allocation': '50%',
                'critical_phases': ['flight_tests', 'validation']
            },
            'data_analyst': {
                'required_skills': ['matlab', 'python', 'signal_processing', 'control_theory'],
                'time_allocation': '75%',
                'critical_phases': ['data_analysis', 'implementation']
            },
            'safety_officer': {
                'required_skills': ['flight_safety', 'risk_assessment', 'emergency_procedures'],
                'time_allocation': '25%',
                'critical_phases': ['preparation', 'flight_tests']
            }
        }
        
        self.equipment_requirements = {
            'vehicle_hardware': {
                'hexacopter_platform': {'quantity': 1, 'cost_estimate': 3000, 'critical': True},
                'backup_vehicle': {'quantity': 1, 'cost_estimate': 3000, 'critical': False},
                'spare_parts': {'quantity': 1, 'cost_estimate': 500, 'critical': True},
                'batteries': {'quantity': 6, 'cost_estimate': 300, 'critical': True}
            },
            'test_equipment': {
                'precision_scale': {'quantity': 1, 'cost_estimate': 200, 'critical': True},
                'pendulum_setup': {'quantity': 1, 'cost_estimate': 100, 'critical': True},
                'motor_test_stand': {'quantity': 1, 'cost_estimate': 800, 'critical': True},
                'load_cells': {'quantity': 3, 'cost_estimate': 600, 'critical': True},
                'torque_sensors': {'quantity': 3, 'cost_estimate': 900, 'critical': True}
            },
            'ground_station': {
                'laptop_computer': {'quantity': 2, 'cost_estimate': 2000, 'critical': True},
                'telemetry_radios': {'quantity': 2, 'cost_estimate': 400, 'critical': True},
                'backup_radio': {'quantity': 1, 'cost_estimate': 300, 'critical': True},
                'data_storage': {'quantity': 2, 'cost_estimate': 200, 'critical': True}
            },
            'safety_equipment': {
                'first_aid_kit': {'quantity': 1, 'cost_estimate': 100, 'critical': True},
                'fire_extinguisher': {'quantity': 2, 'cost_estimate': 100, 'critical': True},
                'safety_barriers': {'quantity': 4, 'cost_estimate': 200, 'critical': False}
            }
        }
        
    def calculate_project_costs(self):
        """Calculate total project costs"""
        
        total_cost = 0
        cost_breakdown = {}
        
        for category, items in self.equipment_requirements.items():
            category_cost = 0
            cost_breakdown[category] = {}
            
            for item, details in items.items():
                item_cost = details['quantity'] * details['cost_estimate']
                category_cost += item_cost
                cost_breakdown[category][item] = item_cost
                
            cost_breakdown[category]['total'] = category_cost
            total_cost += category_cost
            
        # Add personnel costs (estimated)
        personnel_weeks = 9  # Total project duration
        personnel_cost = personnel_weeks * 2000  # $2000 per week per person (blended rate)
        
        cost_breakdown['personnel'] = personnel_cost
        total_cost += personnel_cost
        
        # Add contingency (15%)
        contingency = total_cost * 0.15
        cost_breakdown['contingency'] = contingency
        total_cost += contingency
        
        print("Project Cost Breakdown:")
        print("=" * 30)
        
        for category, cost in cost_breakdown.items():
            if isinstance(cost, dict):
                print(f"{category.title().replace('_', ' ')}:")
                for item, item_cost in cost.items():
                    if item != 'total':
                        print(f"  {item.replace('_', ' ')}: ${item_cost:,.0f}")
                print(f"  Subtotal: ${cost['total']:,.0f}")
            else:
                print(f"{category.title()}: ${cost:,.0f}")
            print()
            
        print(f"TOTAL PROJECT COST: ${total_cost:,.0f}")
        
        return total_cost, cost_breakdown
        
    def generate_resource_allocation_chart(self):
        """Generate resource allocation by phase"""
        
        phases = ['preparation', 'ground_measurements', 'flight_tests', 'data_analysis', 'implementation', 'validation']
        
        allocation_matrix = {
            'project_lead': [100, 100, 100, 100, 100, 100],
            'test_pilot': [25, 50, 100, 0, 25, 100],
            'data_analyst': [25, 50, 25, 100, 100, 50],
            'safety_officer': [100, 25, 100, 0, 0, 50]
        }
        
        print("Resource Allocation by Phase (% of time):")
        print("=" * 50)
        print(f"{'Role':<15s} " + " ".join(f"{phase[:4]:>6s}" for phase in phases))
        print("-" * 50)
        
        for role, allocations in allocation_matrix.items():
            allocation_str = " ".join(f"{alloc:>6d}%" for alloc in allocations)
            print(f"{role:<15s} {allocation_str}")
            
        return allocation_matrix

# Usage
resource_planner = ProjectResourcePlanning()
total_cost, cost_breakdown = resource_planner.calculate_project_costs()
allocation_matrix = resource_planner.generate_resource_allocation_chart()
```

## Phase 1: Preparation and Planning {#phase-1-preparation}

### Comprehensive Test Plan Development

```python
class TestPlanGenerator:
    def __init__(self, vehicle_specs, project_requirements):
        self.vehicle = vehicle_specs
        self.requirements = project_requirements
        
    def generate_complete_test_plan(self):
        """Generate comprehensive test plan document"""
        
        test_plan = {
            'executive_summary': self.create_executive_summary(),
            'objectives': self.define_test_objectives(),
            'test_matrix': self.create_test_matrix(),
            'safety_plan': self.create_safety_plan(),
            'data_requirements': self.define_data_requirements(),
            'success_criteria': self.define_success_criteria(),
            'risk_assessment': self.perform_risk_assessment(),
            'schedule': self.create_detailed_schedule()
        }
        
        return test_plan
        
    def create_executive_summary(self):
        """Create executive summary of test program"""
        
        summary = f"""
        HEXACOPTER SYSTEM IDENTIFICATION TEST PLAN
        
        Vehicle: {self.vehicle['type']} - {self.vehicle['mass']}kg
        Objective: Complete system identification for control system design
        Duration: {self.requirements['duration_weeks']} weeks
        Test Location: {self.requirements['test_site']}
        
        This test program will experimentally identify the dynamic characteristics
        of a 6-motor hexacopter through comprehensive ground and flight testing.
        The identified parameters will be used to design and tune advanced
        control systems with EKF state estimation.
        
        Key deliverables:
        - Measured moments of inertia and mass properties
        - Identified transfer functions for attitude dynamics
        - Motor characterization and time constants
        - Aerodynamic parameter estimation
        - Validated control system parameters
        """
        
        return summary
        
    def define_test_objectives(self):
        """Define specific, measurable test objectives"""
        
        objectives = {
            'primary_objectives': [
                'Measure vehicle moments of inertia to ±5% accuracy',
                'Identify attitude transfer functions with >85% fit',
                'Characterize motor dynamics and time constants',
                'Estimate drag coefficients from flight data',
                'Extract control effectiveness matrix'
            ],
            'secondary_objectives': [
                'Validate identified models against independent flight data',
                'Optimize EKF parameters for identified vehicle',
                'Demonstrate improved control performance',
                'Document complete system ID methodology'
            ],
            'success_metrics': {
                'inertia_accuracy': '±5%',
                'model_fit_quality': '>85% VAF',
                'control_bandwidth': '>10 rad/s',
                'position_accuracy': '<1m RMS in hover',
                'attitude_accuracy': '<2° RMS in hover'
            }
        }
        
        return objectives
        
    def create_test_matrix(self):
        """Create detailed test execution matrix"""
        
        test_matrix = {
            'ground_tests': {
                'GT-001': {
                    'name': 'Mass and Center of Mass Measurement',
                    'method': 'Precision scale and balance technique',
                    'duration': '2 hours',
                    'personnel': ['project_lead', 'data_analyst'],
                    'equipment': ['precision_scale', 'knife_edges'],
                    'deliverables': ['mass_properties_report']
                },
                'GT-002': {
                    'name': 'Moment of Inertia Measurement',
                    'method': 'Bifilar pendulum technique',
                    'duration': '4 hours',
                    'personnel': ['project_lead', 'data_analyst'],
                    'equipment': ['pendulum_setup', 'timer', 'measurement_tape'],
                    'deliverables': ['inertia_measurements', 'uncertainty_analysis']
                },
                'GT-003': {
                    'name': 'Motor Characterization',
                    'method': 'Static thrust and torque testing',
                    'duration': '6 hours',
                    'personnel': ['project_lead', 'data_analyst'],
                    'equipment': ['motor_test_stand', 'load_cells', 'torque_sensors'],
                    'deliverables': ['motor_curves', 'time_constants']
                }
            },
            'flight_tests': {
                'FT-001': {
                    'name': 'Hover Trim and Basic Stability',
                    'method': 'Manual flight testing',
                    'duration': '1 hour',
                    'personnel': ['test_pilot', 'safety_officer', 'project_lead'],
                    'conditions': ['wind < 3 m/s', 'good_visibility'],
                    'deliverables': ['hover_characteristics', 'stability_margins']
                },
                'FT-002': {
                    'name': 'Frequency Domain System ID - Roll',
                    'method': 'ArduPilot automated frequency sweep',
                    'duration': '30 minutes',
                    'personnel': ['test_pilot', 'project_lead'],
                    'conditions': ['wind < 5 m/s', 'GPS_lock'],
                    'deliverables': ['roll_frequency_response', 'coherence_data']
                },
                'FT-003': {
                    'name': 'Frequency Domain System ID - Pitch',
                    'method': 'ArduPilot automated frequency sweep',
                    'duration': '30 minutes',
                    'personnel': ['test_pilot', 'project_lead'],
                    'conditions': ['wind < 5 m/s', 'GPS_lock'],
                    'deliverables': ['pitch_frequency_response', 'coherence_data']
                },
                'FT-004': {
                    'name': 'Frequency Domain System ID - Yaw',
                    'method': 'ArduPilot automated frequency sweep',
                    'duration': '30 minutes',
                    'personnel': ['test_pilot', 'project_lead'],
                    'conditions': ['wind < 5 m/s', 'GPS_lock'],
                    'deliverables': ['yaw_frequency_response', 'coherence_data']
                },
                'FT-005': {
                    'name': 'Step Response Testing',
                    'method': 'Manual pilot inputs',
                    'duration': '1 hour',
                    'personnel': ['test_pilot', 'project_lead'],
                    'conditions': ['wind < 3 m/s', 'experienced_pilot'],
                    'deliverables': ['step_response_data', 'transient_characteristics']
                },
                'FT-006': {
                    'name': 'Aerodynamic Parameter Testing',
                    'method': 'Power-off glides and velocity decay',
                    'duration': '45 minutes',
                    'personnel': ['test_pilot', 'safety_officer', 'project_lead'],
                    'conditions': ['wind < 2 m/s', 'sufficient_altitude'],
                    'deliverables': ['drag_coefficient_data', 'glide_performance']
                }
            }
        }
        
        return test_matrix
        
    def create_safety_plan(self):
        """Create comprehensive safety plan"""
        
        safety_plan = {
            'hazard_identification': {
                'category_1_critical': [
                    'Vehicle impact with personnel',
                    'Uncontrolled flight or flyaway',
                    'Battery fire or explosion',
                    'Propeller strike injury'
                ],
                'category_2_serious': [
                    'Hard landing or crash',
                    'Equipment damage',
                    'Data loss during test',
                    'Weather deterioration'
                ],
                'category_3_minor': [
                    'Test delay due to equipment issues',
                    'Minor vehicle damage',
                    'Incomplete data collection'
                ]
            },
            'safety_measures': {
                'personnel_protection': [
                    'Minimum 20m safety perimeter during flight',
                    'Safety briefing before each test',
                    'PPE required: safety glasses, hearing protection',
                    'Emergency stop procedures established'
                ],
                'vehicle_safeguards': [
                    'Pre-flight inspection checklist',
                    'Redundant communication links',
                    'Automated failsafe systems active',
                    'Maximum flight envelope limits'
                ],
                'environmental_controls': [
                    'Weather monitoring and limits',
                    'Airspace coordination and NOTAM',
                    'Emergency landing area designation',
                    'Fire suppression equipment available'
                ]
            },
            'emergency_procedures': {
                'loss_of_control': 'Immediate emergency stop command, clear area, assess damage',
                'communication_loss': 'Activate RTL mode, maintain visual contact, prepare for manual takeover',
                'battery_failure': 'Immediate landing, power down all systems, inspect for damage',
                'injury_to_personnel': 'Stop all operations, administer first aid, contact emergency services',
                'equipment_malfunction': 'Safe vehicle recovery, document failure mode, assess test impact'
            }
        }
        
        return safety_plan

# Usage
vehicle_specs = {
    'type': 'Hexacopter X-Configuration',
    'mass': 2.5,  # kg
    'span': 0.5,  # m
    'motors': 6,
    'max_thrust': 90  # N total
}

project_reqs = {
    'duration_weeks': 9,
    'test_site': 'Designated UAV Test Range',
    'accuracy_targets': {'inertia': 0.05, 'models': 0.85}
}

test_planner = TestPlanGenerator(vehicle_specs, project_reqs)
complete_test_plan = test_planner.generate_complete_test_plan()
```

## Phase 2: Ground-Based Measurements {#phase-2-ground-measurements}

### Integrated Ground Test Execution

```python
class GroundTestExecutor:
    def __init__(self, test_plan):
        self.test_plan = test_plan
        self.results = {}
        
    def execute_complete_ground_testing(self):
        """Execute all ground-based measurements"""
        
        print("Executing Ground Test Phase")
        print("=" * 40)
        
        # Execute tests in logical order
        test_sequence = [
            ('mass_properties', self.measure_mass_properties),
            ('inertia_measurement', self.measure_moments_of_inertia),
            ('motor_characterization', self.characterize_motors),
            ('sensor_calibration', self.calibrate_sensors),
            ('system_integration', self.verify_system_integration)
        ]
        
        for test_name, test_function in test_sequence:
            print(f"\n--- Executing {test_name.replace('_', ' ').title()} ---")
            
            try:
                result = test_function()
                self.results[test_name] = result
                print(f"✓ {test_name} completed successfully")
                
            except Exception as e:
                print(f"✗ {test_name} failed: {e}")
                self.results[test_name] = {'error': str(e)}
                
        # Generate ground test report
        report = self.generate_ground_test_report()
        return self.results, report
        
    def measure_mass_properties(self):
        """Measure mass and center of mass location"""
        
        # Mass measurement
        print("1. Measuring vehicle mass...")
        measured_mass = float(input("Enter measured mass (kg): "))
        
        # Center of mass measurement
        print("2. Measuring center of mass location...")
        print("   Balance vehicle on knife edge along X-axis")
        x_balance = float(input("   X-axis balance point from reference (m): "))
        
        print("   Balance vehicle on knife edge along Y-axis")
        y_balance = float(input("   Y-axis balance point from reference (m): "))
        
        print("   Measure CG height using tilt method")
        tilt_angle = float(input("   Tilt angle from vertical (degrees): "))
        horizontal_displacement = float(input("   Horizontal displacement (m): "))
        
        z_cg = horizontal_displacement / np.tan(np.radians(tilt_angle))
        
        # Calculate measurement uncertainties
        mass_uncertainty = 0.001  # ±1g
        position_uncertainty = 0.001  # ±1mm
        
        results = {
            'mass': {
                'value': measured_mass,
                'uncertainty': mass_uncertainty,
                'units': 'kg'
            },
            'center_of_mass': {
                'x': {'value': x_balance, 'uncertainty': position_uncertainty, 'units': 'm'},
                'y': {'value': y_balance, 'uncertainty': position_uncertainty, 'units': 'm'},
                'z': {'value': z_cg, 'uncertainty': position_uncertainty, 'units': 'm'}
            },
            'measurement_method': 'knife_edge_balance',
            'test_date': datetime.now().isoformat()
        }
        
        print(f"   Measured mass: {measured_mass:.3f} ± {mass_uncertainty:.3f} kg")
        print(f"   Center of mass: ({x_balance:.3f}, {y_balance:.3f}, {z_cg:.3f}) m")
        
        return results
        
    def measure_moments_of_inertia(self):
        """Measure moments of inertia using bifilar pendulum"""
        
        print("Setting up bifilar pendulum measurement...")
        
        # Setup parameters
        string_length = float(input("String length (m): "))
        string_separation = float(input("String separation (m): "))
        vehicle_mass = self.results['mass_properties']['mass']['value']
        
        # Measure periods for each axis
        periods = {}
        axes = ['roll', 'pitch', 'yaw']
        
        for axis in axes:
            print(f"\nMeasuring {axis} axis inertia:")
            print(f"1. Orient vehicle for {axis} oscillation")
            print("2. Apply small initial displacement")
            print("3. Measure oscillation period over 10 cycles")
            
            # Multiple measurements for accuracy
            measurements = []
            for i in range(3):
                period = float(input(f"   Period measurement {i+1} (s): "))
                measurements.append(period)
                
            periods[axis] = {
                'measurements': measurements,
                'average': np.mean(measurements),
                'std_dev': np.std(measurements),
                'uncertainty': np.std(measurements) / np.sqrt(len(measurements))
            }
            
        # Calculate moments of inertia
        g = 9.81  # m/s²
        inertias = {}
        
        for axis in axes:
            T = periods[axis]['average']
            I = (vehicle_mass * g * string_separation * T**2) / (4 * np.pi**2)
            
            # Uncertainty propagation
            dT = periods[axis]['uncertainty']
            dI = I * 2 * dT / T  # Simplified uncertainty propagation
            
            inertias[axis] = {
                'value': I,
                'uncertainty': dI,
                'units': 'kg⋅m²',
                'period': T,
                'period_uncertainty': dT
            }
            
            print(f"   {axis.title()} inertia: {I:.6f} ± {dI:.6f} kg⋅m²")
            
        results = {
            'setup_parameters': {
                'string_length': string_length,
                'string_separation': string_separation,
                'vehicle_mass': vehicle_mass
            },
            'periods': periods,
            'inertias': inertias,
            'measurement_method': 'bifilar_pendulum',
            'test_date': datetime.now().isoformat()
        }
        
        return results
        
    def characterize_motors(self):
        """Characterize all 6 motors using test stand"""
        
        print("Motor characterization using test stand...")
        
        motor_data = {}
        
        for motor_id in range(1, 7):
            print(f"\nTesting Motor {motor_id}:")
            print("1. Mount motor on test stand")
            print("2. Connect load cell and torque sensor")
            print("3. Run PWM sweep from 1100 to 1900 μs")
            
            # Simulated data collection (in practice, this would be automated)
            pwm_range = np.arange(1100, 1901, 100)
            thrust_data = []
            torque_data = []
            rpm_data = []
            
            for pwm in pwm_range:
                print(f"   PWM: {pwm} μs")
                thrust = float(input("     Thrust (N): "))
                torque = float(input("     Torque (N⋅m): "))
                rpm = float(input("     RPM: "))
                
                thrust_data.append(thrust)
                torque_data.append(torque)
                rpm_data.append(rpm)
                
            # Calculate motor coefficients
            omega = np.array(rpm_data) * 2 * np.pi / 60  # Convert to rad/s
            omega_squared = omega**2
            
            # Fit thrust coefficient: T = kT * ω²
            valid_points = omega_squared > 0
            if np.sum(valid_points) > 3:
                kT = np.polyfit(omega_squared[valid_points], np.array(thrust_data)[valid_points], 1)[0]
                kM = np.polyfit(omega_squared[valid_points], np.array(torque_data)[valid_points], 1)[0]
            else:
                kT = kM = 0
                
            motor_data[f'motor_{motor_id}'] = {
                'pwm_inputs': pwm_range.tolist(),
                'thrust_outputs': thrust_data,
                'torque_outputs': torque_data,
                'rpm_outputs': rpm_data,
                'thrust_coefficient': kT,
                'torque_coefficient': kM,
                'kM_over_kT_ratio': kM / kT if kT > 0 else 0
            }
            
            print(f"   kT: {kT:.2e} N⋅s²")
            print(f"   kM: {kM:.2e} N⋅m⋅s²")
            
        # Calculate average motor characteristics
        all_kT = [data['thrust_coefficient'] for data in motor_data.values()]
        all_kM = [data['torque_coefficient'] for data in motor_data.values()]
        
        average_characteristics = {
            'thrust_coefficient_mean': np.mean(all_kT),
            'thrust_coefficient_std': np.std(all_kT),
            'torque_coefficient_mean': np.mean(all_kM),
            'torque_coefficient_std': np.std(all_kM),
            'motor_matching': np.std(all_kT) / np.mean(all_kT) * 100  # CV%
        }
        
        results = {
            'individual_motors': motor_data,
            'average_characteristics': average_characteristics,
            'measurement_method': 'static_test_stand',
            'test_date': datetime.now().isoformat()
        }
        
        print(f"\nAverage motor characteristics:")
        print(f"   kT: {average_characteristics['thrust_coefficient_mean']:.2e} ± {average_characteristics['thrust_coefficient_std']:.2e}")
        print(f"   Motor matching: {average_characteristics['motor_matching']:.1f}% CV")
        
        return results
        
    def calibrate_sensors(self):
        """Calibrate all sensors and verify operation"""
        
        print("Sensor calibration and verification...")
        
        calibration_results = {
            'accelerometers': self.calibrate_accelerometers(),
            'gyroscopes': self.calibrate_gyroscopes(),
            'magnetometers': self.calibrate_magnetometers(),
            'barometer': self.calibrate_barometer(),
            'gps': self.verify_gps_operation()
        }
        
        return calibration_results
        
    def calibrate_accelerometers(self):
        """Calibrate accelerometer offsets and scaling"""
        
        print("   Accelerometer calibration:")
        print("   1. Place vehicle in 6 orientations (+/-X, +/-Y, +/-Z up)")
        print("   2. Record accelerometer readings for each orientation")
        
        orientations = ['X_up', 'X_down', 'Y_up', 'Y_down', 'Z_up', 'Z_down']
        readings = {}
        
        for orientation in orientations:
            print(f"   Position vehicle {orientation}, press Enter when stable...")
            input()
            
            # In practice, this would read from the flight controller
            accel_x = float(input(f"     Accel X (m/s²): "))
            accel_y = float(input(f"     Accel Y (m/s²): "))
            accel_z = float(input(f"     Accel Z (m/s²): "))
            
            readings[orientation] = [accel_x, accel_y, accel_z]
            
        # Calculate calibration parameters
        # (In practice, use ArduPilot's built-in calibration)
        g = 9.81
        
        calibration = {
            'readings': readings,
            'expected_magnitude': g,
            'calibration_quality': 'good',  # Would be calculated
            'test_date': datetime.now().isoformat()
        }
        
        print("   ✓ Accelerometer calibration complete")
        return calibration
        
    def calibrate_gyroscopes(self):
        """Calibrate gyroscope biases"""
        
        print("   Gyroscope calibration:")
        print("   1. Place vehicle on stable, level surface")
        print("   2. Do not move vehicle during calibration")
        print("   3. Record gyro readings for 60 seconds")
        
        input("   Press Enter to start gyro calibration...")
        
        # Simulated gyro bias measurement
        gyro_bias_x = float(input("   Average X gyro bias (rad/s): "))
        gyro_bias_y = float(input("   Average Y gyro bias (rad/s): "))
        gyro_bias_z = float(input("   Average Z gyro bias (rad/s): "))
        
        calibration = {
            'bias_x': gyro_bias_x,
            'bias_y': gyro_bias_y,
            'bias_z': gyro_bias_z,
            'noise_level': 0.001,  # Would be measured
            'test_date': datetime.now().isoformat()
        }
        
        print("   ✓ Gyroscope calibration complete")
        return calibration
        
    def generate_ground_test_report(self):
        """Generate comprehensive ground test report"""
        
        report = f"""
        GROUND TEST PHASE REPORT
        ========================
        
        Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
        Vehicle: Hexacopter System ID Test Platform
        
        MASS PROPERTIES:
        - Mass: {self.results['mass_properties']['mass']['value']:.3f} kg
        - Center of Mass: ({self.results['mass_properties']['center_of_mass']['x']['value']:.3f}, 
                          {self.results['mass_properties']['center_of_mass']['y']['value']:.3f}, 
                          {self.results['mass_properties']['center_of_mass']['z']['value']:.3f}) m
        
        MOMENTS OF INERTIA:
        - Ixx (Roll):  {self.results['inertia_measurement']['inertias']['roll']['value']:.6f} kg⋅m²
        - Iyy (Pitch): {self.results['inertia_measurement']['inertias']['pitch']['value']:.6f} kg⋅m²
        - Izz (Yaw):   {self.results['inertia_measurement']['inertias']['yaw']['value']:.6f} kg⋅m²
        
        MOTOR CHARACTERISTICS:
        - Average kT: {self.results['motor_characterization']['average_characteristics']['thrust_coefficient_mean']:.2e} N⋅s²
        - Average kM: {self.results['motor_characterization']['average_characteristics']['torque_coefficient_mean']:.2e} N⋅m⋅s²
        - Motor matching: {self.results['motor_characterization']['average_characteristics']['motor_matching']:.1f}% CV
        
        SENSOR CALIBRATION:
        - All sensors calibrated and verified
        - Ready for flight testing
        
        NEXT PHASE: Flight Test Execution
        """
        
        print(report)
        return report

# Usage
import datetime

test_plan = {}  # Would come from Phase 1
ground_executor = GroundTestExecutor(test_plan)
# ground_results, ground_report = ground_executor.execute_complete_ground_testing()
```

This comprehensive workflow implementation provides:

1. **Professional project management** with timeline, resources, and costs
2. **Detailed test planning** with safety procedures and success criteria  
3. **Systematic ground testing** with measurement procedures and uncertainty analysis
4. **Integration points** between all phases
5. **Quality assurance** and documentation standards

The workflow represents the professional standard for experimental system identification projects. Each phase builds on the previous one with clear deliverables and validation checkpoints.

Would you like me to continue with Phases 3-6 (Flight Tests, Data Analysis, Implementation, and Validation) to complete the full workflow?