import numpy as np

class KalmanFilter:
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

class RealTimeTargetTracker:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.kalman_filter = KalmanFilter(initial_state, initial_covariance, process_noise, measurement_noise)

    def update_target(self, measurement):
        A = np.block([
            [np.identity(3), 0.01 * np.identity(3)],
            [np.zeros((3, 3)), np.identity(3)]
        ])
        self.kalman_filter.predict(A)
        self.kalman_filter.update(measurement, np.identity(6))

    def predict_future(self, seconds):
        A = np.block([
            [np.identity(3), seconds * np.identity(3)],
            [np.zeros((3, 3)), np.identity(3)]
        ])
        self.kalman_filter.predict(A)

    def get_target_estimate(self):
        return self.kalman_filter.state[:3]

    def intercept_target(self, target_position, target_velocity):
        # Calculate the relative position and velocity of the target
        relative_position = target_position - self.get_target_estimate()
        relative_velocity = target_velocity - self.kalman_filter.state[3:6]

        # Calculate the time to intercept
        time_to_intercept = np.linalg.norm(relative_position) / np.linalg.norm(relative_velocity)

        # Calculate the interception point
        interception_point = target_position + target_velocity * time_to_intercept

        return interception_point, time_to_intercept


if __name__ == '__main__':
    # Create an instance of the RealTimeTargetTracker
    initial_state = np.hstack([np.array([200.0, 200.0, 200.0]), np.zeros(3)])
    tracker = RealTimeTargetTracker(initial_state, np.identity(6), np.identity(6), np.identity(6))

    # Simulated real-time updates with measurements
    measurements = [
        np.array([200.1, 199.9, 199.8, 19.8, -8.2, 5.1]),  # Replace with your measurements
        np.array([200.2, 199.8, 199.7, 19.7, -8.3, 5.2]),  # Replace with your measurements
        # Add more measurements as needed
    ]

    for measurement in measurements:
        tracker.update_target(measurement)
        estimated_position = tracker.get_target_estimate()
        print(f"Estimated Target Position: {estimated_position}")

    # Calculate the interception point with a given target's position and velocity
    target_position = np.array([300.0, 300.0, 300.0])
    target_velocity = np.array([10.0, -5.0, 2.0])
    interception_point, time_to_intercept = tracker.intercept_target(target_position, target_velocity)

    print(f"Interception Point: {interception_point}")
    print(f"Time to Intercept: {time_to_intercept} seconds")
