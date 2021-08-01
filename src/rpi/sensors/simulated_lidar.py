import numpy as np

from sensors.dummy_lidar import DummyLidar


class SimulatedLidar(DummyLidar):
    """
    Simulated LIDAR Sensor.
    Attributes:
        map_image (numpy.array): Map data for sensor simulation.
        h (int): Height of map_image.
        w (int): Width of map_image.
    """

    def __init__(self, map_image):
        """
        Initialize the Simulated Sensor.
        Args:
            map_image (numpy.array): Map data for sensor simulation.
        """

        self.map_image = map_image
        self.h = map_image.shape[0]
        self.w = map_image.shape[1]

    def get_data(self, Ox=0, Oy=0, Ot=0):
        """
        Get sensor data from a particular origin and orientation.
        Args:
            Ox (int, optional): X-coordinate of origin. Defaults to 0.
            Oy (int, optional): Y-coordinate of origin. Defaults to 0.
            Ot (int, optional): Angle of orientation of sensor, in degrees. Defaults to 0.
        Returns:
            list: Returns point cloud data in polar form.
                `[(r1, t1), (r2, t2), ..., (r359, t359)]`
        """
        out = []
        for i in range(360):
            x, y, r = 0, 0, 0
            theta = np.radians(Ot+i)

            # Checks if search coordinates are within map dimensions
            while Ox+x >= 0 and Oy+y >= 0 and Ox+x < self.w and Oy+y < self.h:
                # Breaks loop if a wall is detected (0)
                if self.map_image[int(Oy+y), int(Ox+x)] == 0:
                    break
                r += 1

                # Polar to Cartesian Conversion
                x = r * np.cos(theta)
                y = r * np.sin(theta)

            t = np.radians(i)
            # Introduce normal error in measurement
            r += np.random.normal(0, 0.5)
            out.append((r, t))
        return out
