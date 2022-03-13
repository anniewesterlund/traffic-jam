import numpy as np

class Car:

    def __init__(self, road, x0=0, v0=0, a0=0, a_max=20, a_min=-150, d_min=5,
                 stop_time=10, driver_restart_response=2, dt=0.01, color=[1, 0, 0]):

        self.road = road    # The road (object) on which the car is driving
        self.position = x0  # The position of the car along the road
        self.velocity = v0  # The car velocity
        self.acceleration = a0  # The acceleration velocity
        self.maximum_acceleration = a_max # Maximum allowed velocity
        self.minimum_acceleration = a_min # Decceleration at the start of braking

        self.stop_time = stop_time  # The time needed for the driver to stop before the minimum allowed distance to car in front.
        self.driver_restart_response = driver_restart_response  # Min time before driver restarts
        self.has_obstacle_ahead = False # Indicates whether or not the car in front is deemed close enough.
        self.n_iterations_braking = self.driver_restart_response / dt   # Nr of iterations braking (counter).
        self.obstacle_distance = 0  # Distance to car in front
        self.obstacle_velocity = 0  # Velocity of car in front
        self.min_obstacle_distance = d_min  # Minimum allowed distance to car in front

        self.dt = dt    # Time step in integration
        self.length = 3 # Length of a car

        self.visualization_data = None # The data used for plotting the car.
        self.color = color  # Color of the car when plotting
        return

    def get_parameters(self):
        """
        Return a dictionary with the current car parameters.
        :return:
        """
        parameters = {
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'a_max': self.maximum_acceleration,
            'a_min': self.minimum_acceleration,
            'd_min': self.min_obstacle_distance,
            'stop_time': self.stop_time,
            'driver_restart_response': self.driver_restart_response,
            'dt': self.dt
        }
        return parameters

    def draw(self, ax):
        """
        Drawing the car using its direction (yaw) and position along the road.
        :param ax:
        :return:
        """
        x_mid, y_mid, yaw_angle = self.road.get_car_pose_from_distance(self.position)

        # Calculating positions of front and rear of car
        x = np.array([x_mid - self.length / 2 * np.cos(yaw_angle),
                      x_mid + self.length / 2 * np.cos(yaw_angle)])

        y = np.array([y_mid - self.length / 2 * np.sin(yaw_angle),
                      y_mid + self.length / 2 * np.sin(yaw_angle)])

        if self.visualization_data is None:
            self.visualization_data, = ax.plot(x, y, color=self.color, linewidth=7)
        else:
            self.visualization_data.set_xdata(x)
            self.visualization_data.set_ydata(y)
        return ax

    def update_position(self):
        """
        Updating car position given current acceleration and velocity.
        :return:
        """
        # Set car acceleration depending on current velocity and speed limit of the road.
        self.acceleration = self.maximum_acceleration * (1-self.velocity/self.road.speed_limit)

        if self.has_obstacle_ahead:
            # If obstacle ahead => push brakes by using a negative acceleration and
            # scaling using distance to obstacle.
            self.n_iterations_braking += 1
            self.acceleration = self.minimum_acceleration * self.min_obstacle_distance / self.obstacle_distance

        # Update car velocity
        self.velocity = self.velocity + self.dt * self.acceleration
        if self.velocity < 0:
            # Do not allow negative velocities.
            self.velocity = 0
            self.acceleration = 0

        # Update car position
        self.position = self.position + self.dt * self.velocity + (self.dt ** 2)/2 * self.acceleration
        return

    def check_for_obstacle(self, object):
        """
        Checking whether the obstacle in front is close enough for the car to push the brakes.
        :param object:
        :return:
        """

        # Calculate distance to obstacle in front along road
        self.obstacle_distance = self.road.periodic_distance(object, self)

        # Calculate difference in velocity between the car and the obstacle in front
        self.obstacle_velocity = object.velocity
        delta_velocity = self.velocity - self.obstacle_velocity

        if self.obstacle_distance < self.stop_time * delta_velocity + self.min_obstacle_distance:
            # If the obstacle is closer than the safety distance, then the driver should push the brakes.
            self.has_obstacle_ahead = True
            self.n_iterations_braking = 0
        elif self.n_iterations_braking >= self.driver_restart_response / self.dt:
            # If the obstacle is not within safety distance and the driver pushed the brakes for long enough,
            # then the driver should stop pushing the brakes.
            self.has_obstacle_ahead = False
        return