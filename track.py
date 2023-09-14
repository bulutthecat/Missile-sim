import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter:
    """
    : Class VARIABLES
    : self.state: State of the system
    : self.covariance: Covariance of the system
    : self.process_noise: Process noise
    : self.measurement_noise: Measurement noise
    : return: None
    """
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self, A):
        self.state = A @ self.state
        self.covariance = A @ self.covariance @ A.T + self.process_noise

    def update(self, z, H):
        y = z - H @ self.state
        S = H @ self.covariance @ H.T + self.measurement_noise
        K = self.covariance @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        I = np.identity(self.state.shape[0])
        self.covariance = (I - K @ H) @ self.covariance


def initialize_constants():
    return {
        'initial_target_position': np.array([200.0, 200.0, 200.0]),
        'initial_target_velocity': np.array([20.0, -8.0, 5.0]),
        'projectile_position': np.array([0.0, 0.0, 0.0]),
        'projectile_velocity': np.array([50.0, 50.0, 50.0]),
        'Cd': 0.1,
        'mass': 1.0,
        'burn_time': 2.0,
        'max_speed': 100.0,
        'initial_speed': 10.0,
        'dt': 0.01,
        'T': 9.0,
        'downward_accel': -5.0,
        'fov_up_down': np.radians(50),
        'air_density': 1.225,
        'cross_section': 0.01 # m^2, update this value with a more accurate one
    }


def simulate_movement(params):
    """
    : Function VARIABLES
    : params['initial_target_position']: Initial position of the target
    : params['initial_target_velocity']: Initial velocity of the target
    : params['projectile_position']: Initial position of the Missile
    : params['projectile_velocity']: Initial velocity of the Missile
    : params['Cd']: Drag coefficient of the Missile
    : params['mass']: Mass of the Missile
    : params['burn_time']: Time for which the Missile burns
    : params['max_speed']: Maximum speed of the Missile
    : params['initial_speed']: Initial speed of the Missile
    : params['dt']: Time step
    : params['T']: Total time for which the simulation runs
    : params['downward_accel']: Acceleration due to gravity
    : params['fov_up_down']: Field of view of the Missile
    : params['air_density']: Air density
    : params['cross_section']: Cross section of the Missile
    : return: Target positions, Missile positions, Interception time, Burn time position
    """
    gravitational_accel = np.array([0.0, 0.0, -9.81])
    burn_time_position = None
    interception_radius = 1.0
    interception_time = None

    initial_state = np.hstack([params['initial_target_position'], params['initial_target_velocity']])
    kf = KalmanFilter(initial_state, np.identity(6), np.identity(6), np.identity(6))
    projectile_velocity = params['projectile_velocity'] / np.linalg.norm(params['projectile_velocity']) * params['initial_speed']
    target_positions, projectile_positions = [], []

    A = np.block([
        [np.identity(3), params['dt'] * np.identity(3)],
        [np.zeros((3, 3)), np.identity(3)]
    ])

    def get_wind_vector(t):
        """
        : Function VARIABLES
        : t: Time
        : return: Wind vector at time t
        """
        return np.array([
            0.1 * np.sin(0.1 * t),
            0.1 * np.cos(0.1 * t),
            0
        ])
    
    for t in np.arange(0, params['T'], params['dt']):
        params['initial_target_velocity'] += np.random.normal(0, 0.7, size=3)
        params['initial_target_velocity'][2] += params['downward_accel'] * params['dt']
        params['initial_target_position'] += params['initial_target_velocity'] * params['dt']
        z = np.hstack([params['initial_target_position'], params['initial_target_velocity']]) + np.random.normal(0, 0.1, size=6)

        kf.predict(A)
        kf.update(z, np.identity(6))

        estimated_target_position = kf.state[:3]
        desired_direction = estimated_target_position - params['projectile_position']
        desired_direction /= np.linalg.norm(desired_direction)
        forward_direction = projectile_velocity / np.linalg.norm(projectile_velocity)

        angle_to_target = np.arccos(np.dot(forward_direction, desired_direction))
        
        if t < params['burn_time']:
            current_speed = params['initial_speed'] + t * (params['max_speed'] - params['initial_speed']) / params['burn_time']
        else:
            current_speed = params['max_speed']
            wind_vector = get_wind_vector(t)
            wind_force = -0.5 * params['Cd'] * params['air_density'] * params['cross_section'] * np.linalg.norm(wind_vector) * wind_vector
            drag_force = -0.5 * params['Cd'] * params['air_density'] * params['cross_section'] * np.linalg.norm(projectile_velocity) * projectile_velocity
            net_force = drag_force + params['mass'] * gravitational_accel + wind_force # Add wind_force here
            projectile_velocity += (net_force / params['mass']) * params['dt']


        if np.abs(angle_to_target) < params['fov_up_down']:
            steer_strength = 0.1
            new_direction = (1 - steer_strength) * forward_direction + steer_strength * desired_direction
            projectile_velocity = new_direction * current_speed

        params['projectile_position'] += projectile_velocity * params['dt']
        
        # Account for gravitational effect on missile (not on target)
        params['projectile_position'] += 0.5 * gravitational_accel * params['dt'] ** 2
        projectile_velocity += gravitational_accel * params['dt']

        target_positions.append(params['initial_target_position'].copy())
        projectile_positions.append(params['projectile_position'].copy())
        distance_to_target = np.linalg.norm(params['projectile_position'] - params['initial_target_position'])
        
        if t == params['burn_time']:
            burn_time_position = params['projectile_position'].copy()

        if distance_to_target < interception_radius:
            print(f"Target intercepted at time {t} seconds!")
            interception_time = t
            plot_paths(np.array(target_positions), np.array(projectile_positions), burn_time_position, params['projectile_position'])
            break

    return np.array(target_positions), np.array(projectile_positions), interception_time, burn_time_position

def plot_paths(target_positions, projectile_positions, burn_time_position, interception_position):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(target_positions[:, 0], target_positions[:, 1], target_positions[:, 2], label='Target Path')
    ax.plot(projectile_positions[:, 0], projectile_positions[:, 1], projectile_positions[:, 2], label='Projectile Path', linestyle='dashed')

    if burn_time_position is not None:
        ax.scatter(burn_time_position[0], burn_time_position[1], burn_time_position[2], c='red', marker='x', label='Burn Time Stop')
        
    if interception_position is not None:
        ax.scatter(interception_position[0], interception_position[1], interception_position[2], c='green', marker='o', label='Interception Point')

    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

if __name__ == '__main__':
    params = initialize_constants()
    target_positions, projectile_positions, interception_time, burn_time_position = simulate_movement(params)
    interception_position = projectile_positions[-1] if interception_time is not None else None
    
    if interception_time is not None:
        print(f"Interception time: {interception_time} seconds")
        plot_paths(target_positions, projectile_positions, burn_time_position, interception_position)
        has_intercepted = True
    else:
        print("No interception occurred.")
        has_intercepted = False
    
    if not has_intercepted:
        plot_paths(target_positions, projectile_positions, burn_time_position, interception_position)