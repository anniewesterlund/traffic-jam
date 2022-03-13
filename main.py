import traffic_simulation as ts
import numpy as np
import matplotlib.pyplot as plt

def main():

    # Setting keyword arguments to the car class - set to the default values.
    car_kwargs = {
        'v0': 0,                            # Initial velocity
        'a0': 0,                            # Initial acceleration
        'a_max': 20,                        # Maximum acceleration
        'a_min': -150,                      # Minimum acceleration/maximum deceleration before reaching the safety distance to car in front.
        'd_min': 5,                         # Minimum distance to keep to the car in front.
        'stop_time': 10,                    # The time needed to stop before d_min is reached. Is multiplied by velocity difference.
        'driver_restart_response': 2,       # The time delay for the driver to start driving forward after pushing the brakes.
        'dt': 0.01                          # Time step used in integration.
    }

    # Setting keyword arguments to the Simulator class
    kwargs = {
        'n_cars': 25,
        'road': ts.CircleRoad(speed_limit=60, radius=300), # Selecting road and setting its properties (Note: I only implemented a circular road for now)
        'car_kwargs': car_kwargs,
        'n_iterations': 10000,
        'visualize_every': 40, # How often to update plotting (every n:th iteration).
        'log_file': 'traffic_jam.log'
    }

    # Creating simulation object and running simulation
    simulation = ts.TrafficJamSimulator(**kwargs)
    simulation.run()
    return

if __name__ == '__main__':
    main()