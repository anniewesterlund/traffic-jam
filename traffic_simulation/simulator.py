import numpy as np
import matplotlib.pyplot as plt
import traffic_simulation as ts

class TrafficJamSimulator:

    def __init__(self, n_cars=1, road=None, car_kwargs={}, n_iterations=5000, visualize_every=10, log_file='log.log'):

        self.n_cars = n_cars
        self.n_iterations = n_iterations
        self.cars = []

        self.car_visualizations = []
        self.visualize_every = visualize_every

        self.road = road
        if road is None:
            self.road = ts.CircleRoad(radius=100)

        starting_positions = self.road.generate_initial_positions(n_cars)

        # The first car will brake during the simulation and will be drawn in blue
        self.cars.append(ts.Car(self.road, x0=starting_positions[0], **car_kwargs, color=[0, 0, 1]))
        for i_car in range(1, n_cars):
            self.cars.append(ts.Car(self.road, x0=starting_positions[i_car], **car_kwargs))

        # If a car is closer than crash distance to the car in front, it is considered crashed.
        self.crash_distance = 0.5
        # The iteration when the first car is forced to brake.
        self.iteration_forced_brake = 600

        self.log_file = log_file

        # Write specified parameters to file
        init_message = '# ========================================================================================== #\n' + \
                       '# Initializing simulation of traffic jam.  \n' + \
                       '# ========================================================================================== #\n#\n' + \
                       '# Visualizing simulations every ' + str(visualize_every) + 'th iteration. \n#\n' + \
                       '# Number of cars in simulation: '+str(n_cars) + '\n' +\
                       '# Number of iterations: '+str(n_iterations)+'\n#\n' + \
                       '# Road parameters: \n# ' + \
                       str(self.road.get_parameters()) + '\n#\n' + \
                       '# Initial car parameters: \n# ' + \
                       str(self.cars[-1].get_parameters()) +'\n#\n'+ \
                       '# Forcing the first car to push brakes at iteration ' + \
                       str(self.iteration_forced_brake) + '.\n' + \
                       '# ========================================================================================== #\n#\n'

        self.write_to_log(init_message, mode='w')
        return

    def write_to_log(self, message, mode='a'):
        """
        Writing message to log.
        :param message:
        :param mode:
        :return:
        """
        with open(self.log_file, mode) as file_id:
            file_id.write(message)
        return

    def draw_cars(self, ax):
        """
        Plot updated car positions on the road.
        :param ax:
        :return:
        """
        for car in self.cars:
            ax = car.draw(ax)
        return ax

    def update_positions(self):
        """
        Update the positions of all cars in the simulation.
        :return:
        """
        has_crashed = False
        crashed_cars = []
        for i_car, car in enumerate(self.cars):
            # Check if the car in front is closer than the safety distance.
            index_car_in_front = np.mod(i_car+1, self.n_cars)
            car.check_for_obstacle(self.cars[index_car_in_front])

            # Check if the car crashed into the front car.
            if car.obstacle_distance <= self.crash_distance:
                crashed_cars.append(i_car)
                has_crashed = True

            # Update car position along the road
            car.update_position()

        if has_crashed:
            self.write_to_log('# Detected crashed car(s) (ID: '+str(crashed_cars)+'). \n# Simulation stopped.')

        return has_crashed

    def log_data(self, i_iteration):
        """
        Logging data of the first car (position, velocity, distance to car in front).
        :return:
        """
        time = self.cars[0].dt * i_iteration
        p0 = self.cars[0].position
        v0 = self.cars[0].velocity
        a0 = self.cars[0].acceleration
        obstacle_distance0 = self.cars[0].obstacle_distance

        message = "%6.2f %18.3f %20.3f %24.3f %20.3f \n" % (time, p0, v0, a0, obstacle_distance0)
        self.write_to_log(message)
        return

    def run(self):
        """
        Function for launching and running a traffic jam simulation.
        :return:
        """
        self.write_to_log('# Starting simulation. \n#\n')

        message = '# ========================================================================================== #\n' + \
                  '# Time\t Position (car 0)\t Velocity (car 0)\t Acceleration (car 0)\t Obstacle_distance (car 0) \n' + \
                  '# ========================================================================================== #\n'
        self.write_to_log(message)

        plt.ion()

        fig = plt.figure(1)
        ax = fig.subplots(1, 1)

        ax = self.road.draw(ax)
        ax = self.road.draw(ax)

        has_crashed = False
        for i_iteration in range(self.n_iterations):

            # Force the first car to push the brakes after 600 steps
            if i_iteration == self.iteration_forced_brake:
                self.cars[0].has_obstacle_ahead = True
                self.cars[0].n_iterations_braking = 0

            # Update car positions
            has_crashed = self.update_positions()

            # Log data from current time step
            self.log_data(i_iteration)

            # Visualize car positions every self.visualize_every:th step
            if np.mod(i_iteration+1, self.visualize_every) == 0 or has_crashed:
                ax = self.draw_cars(ax)
                fig.canvas.draw_idle()
                plt.pause(1e-3)

            if has_crashed:
                # If a car crashed into the front car, then we stop the simulation.
                break

        if not has_crashed:
            self.write_to_log('# Simulation finished.')
        return

