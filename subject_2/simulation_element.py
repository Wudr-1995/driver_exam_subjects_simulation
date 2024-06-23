import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

class ReverseRelatedMethod:
    def __init__(self):
        pass

    @staticmethod
    def get_position_from_status(status):
        # status: [x, y, yaw]
        return np.array([status[0], status[1]])

    @staticmethod
    def get_direction_from_status(status):
        return np.array([np.cos(np.deg2rad(status[2])), np.sin(np.deg2rad(status[2]))])

    @staticmethod
    def get_n_from_status(status, sign=1):
        # left reverse: sign = 1; right reverse: sign = -1
        return np.array([-sign * np.sin(np.deg2rad(status[2])), sign * np.cos(np.deg2rad(status[2]))])

    @staticmethod
    def get_parallel_pos(pos, n, d):
        return pos + d * n;

    @staticmethod
    def plot_trajectory(point1, point2, point3, point4, center, radius):
        plt.plot([point1[0], point2[0]], [point1[1], point2[1]], 'b-', label='line segment 1')

        # get start angle of the curve
        start_angle = np.arctan2(point2[1] - center[1], point2[0] - center[0])
        end_angle = np.arctan2(point3[1] - center[1], point3[0] - center[0])

        # make sure the direction of the angles
        if start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle

        angles = np.linspace(start_angle, end_angle, 100)
        arc_x = center[0] + radius * np.cos(angles)
        arc_y = center[1] + radius * np.sin(angles)

        # draw the arc segment
        plt.plot(arc_x, arc_y, 'r-', label='Arc Segment')

        # draw the straight line from the end of the arc to the end position
        plt.plot([point3[0], point4[0]], [point3[1], point4[1]], 'g-', label='Line Segment 2')

        # show the points
        plt.scatter([point1[0], point2[0], point3[0], point4[0]], [point1[1], point2[1], point3[1], point4[1]], color='black')

        # show the trajectory
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Trajectory Plot')
        plt.show()

class Car:
    def __init__(self, length, width, min_r):
        self.length = length
        self.width = width
        self.min_r = min_r
        self.max_c = 1. / min_r

    def get_width(self):
        return self.width

    def get_hight(self):
        return self.hight

    def get_minr(self):
        return min_r

    def get_maxc(self):
        return max_c

class ReverseGarage:
    def __init__(self, car_length, car_width):
        self.s = car_length * 1.5
        self.l = car_length + 0.7
        self.w = car_width + 0.6
        self.h = self.s
        self.space_edge = ((0, self.l + self.s), (self.h * 2 + self.w, self.l + self.s),\
                        (0, self.l), (self.h, self.l),\
                        (self.h, self.l), (self.h, 0),\
                        (self.h, 0), (self.h + self.w, 0),\
                        (self.h + self.w, self.l), (self.h + self.w, 0),\
                        (self.h + self.w, self.l), (self.h * 2 + self.w, self.l))

    def get_s(self):
        return self.s

    def get_l(self):
        return self.l

    def get_w(self):
        return self.w

    def get_h(self):
        return self.h
