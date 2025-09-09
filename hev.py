import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1  # Time step (seconds)
sim_time = 60  # Total simulation time (seconds)

# Vehicle parameters
vehicle_mass = 1500  # kg
air_resistance = 0.32  # drag coefficient
frontal_area = 2.2  # m^2
rolling_resistance = 0.015
gravity = 9.81  # m/s^2
air_density = 1.225  # kg/m^3
max_engine_force = 4000  # N
max_motor_force = 2000  # N

# Cruise control parameters
target_speed = 27.78  # m/s (100 km/h)
Kp = 500
Ki = 40

# State variables
speed = 0.0
position = 0.0
integral_error = 0.0

# Data logging
time_data = []
speed_data = []
engine_force_data = []
motor_force_data = []

for t in np.arange(0, sim_time, dt):
    # Calculate total resistance
    F_drag = 0.5 * air_resistance * air_density * frontal_area * speed ** 2
    F_roll = rolling_resistance * vehicle_mass * gravity
    total_resistance = F_drag + F_roll

    # Cruise control PI controller
    error = target_speed - speed
    integral_error += error * dt
    total_force = Kp * error + Ki * integral_error

    # Split force between engine and motor (simple heuristic: use motor for low loads)
    if total_force < max_motor_force:
        motor_force = np.clip(total_force, 0, max_motor_force)
        engine_force = 0
    else:
        motor_force = max_motor_force
        engine_force = np.clip(total_force - max_motor_force, 0, max_engine_force)

    # Net force applied to the vehicle
    drive_force = engine_force + motor_force
    net_force = drive_force - total_resistance
    acceleration = net_force / vehicle_mass

    # Update speed and position
    speed += acceleration * dt
    speed = max(0, speed)
    position += speed * dt

    # Log data
    time_data.append(t)
    speed_data.append(speed)
    engine_force_data.append(engine_force)
    motor_force_data.append(motor_force)

# Plot results
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time_data, speed_data)
plt.axhline(target_speed, color='r', linestyle='--', label='Target Speed')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.title('HEV Cruise Control Simulation')

plt.subplot(2, 1, 2)
plt.plot(time_data, engine_force_data, label='Engine Force')
plt.plot(time_data, motor_force_data, label='Motor Force')
plt.ylabel('Force (N)')
plt.xlabel('Time (s)')
plt.legend()

plt.tight_layout()
plt.show()