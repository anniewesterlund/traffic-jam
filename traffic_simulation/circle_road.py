import numpy as np

class CircleRoad:

    def __init__(self, speed_limit=30, radius=100, thickness=10):
        self.radius = radius
        self.name = 'Circle road'

        self.thickness = thickness
        self.path_length = 2 * np.pi * self.radius
        self.speed_limit = speed_limit
        return

    def get_parameters(self):
        """
        Return a dictionary with the road parameters.
        :return:
        """
        parameters = {
            'name': self.name,
            'radius': self.radius,
            'thickness': self.thickness,
            'speed_limit': self.speed_limit
        }
        return parameters

    def periodic_distance(self, object1, object2):
        """
        Compute the (periodic) distance between two objects given their position along the road and lengths.
        :param object1:
        :param object2:
        :return:
        """
        distance_raw = (object1.position - object1.length / 2) - (object2.position + object2.length / 2)
        angle = distance_raw / self.radius
        angle_wrapped = np.mod(angle, 2*np.pi)
        distance = angle_wrapped * self.radius
        return distance

    def draw(self, ax):
        """
        Plot the road given its radius and thickness.
        :param ax:
        :return:
        """
        angles = np.linspace(0, 2 * np.pi, 1000)

        x_coordinates_inner = (self.radius - self.thickness/2) * np.cos(angles)
        y_coordinates_inner = (self.radius - self.thickness/2) * np.sin(angles)

        x_coordinates_outer = (self.radius + self.thickness/2) * np.cos(angles)
        y_coordinates_outer = (self.radius + self.thickness/2) * np.sin(angles)

        ax.plot(x_coordinates_inner, y_coordinates_inner, linewidth=2, color=[0, 0, 0])
        ax.plot(x_coordinates_outer, y_coordinates_outer, linewidth=2, color=[0, 0, 0])

        ax.axis('equal')
        ax.axis('off')
        return ax

    def generate_initial_positions(self, n_cars):
        """
        Distribute cars equidistantly along the full path.
        :param n_cars:
        :return:
        """
        x0 = np.linspace(0, self.path_length-self.path_length/n_cars, n_cars)
        return x0

    def get_car_pose_from_distance(self, distance_along_path):
        """
        Compute coordinates and yaw-angle of car given its position along the road.
        :param distance_along_path:
        :return:
        """
        angle_travelled = distance_along_path/self.radius
        x = self.radius * np.cos(angle_travelled)
        y = self.radius * np.sin(angle_travelled)
        yaw = angle_travelled + np.pi/2
        return x, y, yaw