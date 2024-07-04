import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class PathPlanningRelatedMethod:
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
    def plot_trajectory():
        pass

class ReverseRelatedMethod(PathPlanningRelatedMethod):
    def __init__(self):
        pass

    @staticmethod
    def plot_trajectory(ax, points, center, radius, alpha=1.0):
        # get start angle of the curve
        start_angle = np.arctan2(points[1][1] - center[1], points[1][0] - center[0])
        end_angle = np.arctan2(points[2][1] - center[1], points[2][0] - center[0])

        # make sure the direction of the angles
        if start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle

        angles = np.linspace(start_angle, end_angle, 100)
        arc_x = center[0] + radius * np.cos(angles)
        arc_y = center[1] + radius * np.sin(angles)

        # draw the straight line from the start position to the start of the arc
        line_0, = ax.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], 'b-', label='line segment 1', alpha=alpha)

        # draw the arc segment
        arc_0, = ax.plot(arc_x, arc_y, 'r-', label='Arc Segment', alpha=alpha)

        # draw the straight line from the end of the arc to the end position
        line_1, = ax.plot([points[2][0], points[3][0]], [points[2][1], points[3][1]], 'g-', label='Line Segment 2', alpha=alpha)

        # show the points
        # ax.scatter([points[0][0], points[1][0], points[2][0], points[3][0]], [points[0][1], points[1][1], points[2][1], points[3][1]], color='black', alpha=alpha)

        # show the trajectory
        # ax.legend()

        return line_0, arc_0, line_1

class ReverseTrajectoryHandler(ReverseRelatedMethod):
    def __init__(self, ax, points, center, radius, alpha=1.0):
        self.line_0, self.arc_0, self.line_1 = self.plot_trajectory(ax, points, center, radius, alpha)
        self.center = center
        self.radius = radius
        self.points = points

    def update_status(self, points, center, radius):
        self.points = points
        self.center = center
        self.radius = radius

    def update(self, frame):
        self.line_0.set_xdata([self.points[0][0], self.points[1][0]])
        self.line_0.set_ydata([self.points[0][1], self.points[1][1]])
        self.line_1.set_xdata([self.points[2][0], self.points[3][0]])
        self.line_1.set_ydata([self.points[2][1], self.points[3][1]])

        start_angle = np.arctan2(self.points[1][1] - self.center[1], self.points[1][0] - self.center[0])
        end_angle = np.arctan2(self.points[2][1] - self.center[1], self.points[2][0] - self.center[0])

        # make sure the direction of the angles
        if start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle

        angles = np.linspace(start_angle, end_angle, 100)
        arc_x = self.center[0] + self.radius * np.cos(angles)
        arc_y = self.center[1] + self.radius * np.sin(angles)

        self.arc_0.set_xdata(arc_x)
        self.arc_0.set_ydata(arc_y)

        return [self.line_0, self.arc_0, self.line_1]

class Car:
    def __init__(self, theta, x, y, car_type='weight_center', length=4.6, width=1.8, min_r=5, dis_axles=2.6, max_yaw=30):

        # theta: the posture of the body of the car
        # x: the x coordinate of the car
        # y: the y coordinate of the car
        # car_type: 'weight_center', the turnning center of the car is determined by the weight center which is the default option
        # car_type: 'rear_axle_center', the turnning center of the car is determined by the rear axle center
        # length: the length of the car
        # width: the width of the car
        # min_r: the minimum turnning radius of the car, only work as the 'weight_center' type
        # dis_axles: the distance between two axles of the car
        # max_yaw: the maximum yaw angle of the car

        if car_type != 'weight_center' and car_type != 'rear_axle_center':
            print("wrong value of car_type ('weight_center' and 'rear_axle_center' only)")
            return None

        self.theta = np.deg2rad(theta)
        self.x = x
        self.y = y

        self.length = length
        self.width = width
        self.min_r = min_r # only for the weight_center model, a coarse model
        self.max_c = 1. / min_r

        # for rear_axle center model, a fine model
        self.car_type = car_type
        self.dis_axles = dis_axles
        self.dis_wheels = width
        self.max_yaw = np.deg2rad(max_yaw)
        self.rear_min_r = self.dis_axles / np.tan(self.max_yaw)
        self.front_rear_min_r = self.dis_axles / np.sin(self.max_yaw)

        self.efficient_min_r = self.min_r if self.car_type == 'weight_center' else self.rear_min_r

        # moving parameters
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.turn_left = 0
        self.turn_right = 0
        self.reversing_status = 1

        # for animation
        self.line_0 = None
        self.line_1 = None
        self.line_2 = None
        self.line_3 = None
        self.arrow = None

        # for recording the step status of the Car
        self.step_status = 0

    def get_vertex(self):
        vertexs = None
        arrow = None
        wheels = None
        if self.car_type == 'weight_center':
            vertexs, arrow, wheels = self.get_vertex_according_to_weight_center()
        else:
            vertexs, arrow, wheels = self.get_vertex_according_to_rear_axle_center()
        return vertexs, arrow, wheels

    def get_vertex_according_to_weight_center(self):
        dir_x = np.cos(self.theta)
        dir_y = np.sin(self.theta)
        n_x = -dir_y
        n_y = dir_x

        arr_st_x = self.x - dir_x * self.length * 0.3 # arrow start position
        arr_st_y = self.y - dir_y * self.length * 0.3
        arr_ed_x = self.x + dir_x * self.length * 0.3 # arrow end position
        arr_ed_y = self.y + dir_y * self.length * 0.3

        mid_edge_0_x = self.x - n_x * self.width * 0.5
        mid_edge_0_y = self.y - n_y * self.width * 0.5
        mid_edge_1_x = self.x + n_x * self.width * 0.5
        mid_edge_1_y = self.y + n_y * self.width * 0.5

        vertex_0_x = mid_edge_0_x + dir_x * self.length * 0.5
        vertex_0_y = mid_edge_0_y + dir_y * self.length * 0.5
        vertex_1_x = mid_edge_0_x - dir_x * self.length * 0.5
        vertex_1_y = mid_edge_0_y - dir_y * self.length * 0.5
        vertex_2_x = mid_edge_1_x - dir_x * self.length * 0.5
        vertex_2_y = mid_edge_1_y - dir_y * self.length * 0.5
        vertex_3_x = mid_edge_1_x + dir_x * self.length * 0.5
        vertex_3_y = mid_edge_1_y + dir_y * self.length * 0.5

        wheel_0_x = mid_edge_0_x + dir_x * self.dis_axles * 0.5
        wheel_0_y = mid_edge_0_y + dir_y * self.dis_axles * 0.5
        wheel_1_x = mid_edge_0_x - dir_x * self.dis_axles * 0.5
        wheel_1_y = mid_edge_0_y - dir_y * self.dis_axles * 0.5
        wheel_2_x = mid_edge_1_x - dir_x * self.dis_axles * 0.5
        wheel_2_y = mid_edge_1_y - dir_y * self.dis_axles * 0.5
        wheel_3_x = mid_edge_1_x + dir_x * self.dis_axles * 0.5
        wheel_3_y = mid_edge_1_y + dir_y * self.dis_axles * 0.5

        arrow = ((arr_st_x, arr_st_y),
                 (arr_ed_x, arr_ed_y))

        vertexs = ((vertex_0_x, vertex_0_y),
                   (vertex_1_x, vertex_1_y),
                   (vertex_2_x, vertex_2_y),
                   (vertex_3_x, vertex_3_y),
                   (vertex_0_x, vertex_0_y))

        wheels = ((wheel_0_x, wheel_0_y),
                  (wheel_1_x, wheel_1_y),
                  (wheel_2_x, wheel_2_y),
                  (wheel_3_x, wheel_3_y))

        return vertexs, arrow, wheels

    def get_vertex_according_to_rear_axle_center(self):
        dir_x = np.cos(self.theta)
        dir_y = np.sin(self.theta)
        n_x = -dir_y
        n_y = dir_x

        arr_st_x = self.x
        arr_st_y = self.y
        arr_ed_x = self.x + dir_x * self.dis_axles
        arr_ed_y = self.y + dir_y * self.dis_axles

        mid_edge_0_x = self.x + self.dis_axles * dir_x / 2 + self.dis_wheels * n_x / 2
        mid_edge_0_y = self.y + self.dis_axles * dir_y / 2 + self.dis_wheels * n_y / 2
        mid_edge_1_x = self.x + self.dis_axles * dir_x / 2 - self.dis_wheels * n_x / 2
        mid_edge_1_y = self.y + self.dis_axles * dir_y / 2 - self.dis_wheels * n_y / 2

        vertex_0_x = mid_edge_0_x + dir_x * self.length * 0.5
        vertex_0_y = mid_edge_0_y + dir_y * self.length * 0.5
        vertex_1_x = mid_edge_0_x - dir_x * self.length * 0.5
        vertex_1_y = mid_edge_0_y - dir_y * self.length * 0.5
        vertex_2_x = mid_edge_1_x - dir_x * self.length * 0.5
        vertex_2_y = mid_edge_1_y - dir_y * self.length * 0.5
        vertex_3_x = mid_edge_1_x + dir_x * self.length * 0.5
        vertex_3_y = mid_edge_1_y + dir_y * self.length * 0.5

        wheel_0_x = mid_edge_0_x + dir_x * self.dis_axles * 0.5
        wheel_0_y = mid_edge_0_y + dir_y * self.dis_axles * 0.5
        wheel_1_x = mid_edge_0_x - dir_x * self.dis_axles * 0.5
        wheel_1_y = mid_edge_0_y - dir_y * self.dis_axles * 0.5
        wheel_2_x = mid_edge_1_x - dir_x * self.dis_axles * 0.5
        wheel_2_y = mid_edge_1_y - dir_y * self.dis_axles * 0.5
        wheel_3_x = mid_edge_1_x + dir_x * self.dis_axles * 0.5
        wheel_3_y = mid_edge_1_y + dir_y * self.dis_axles * 0.5

        arrow = ((arr_st_x, arr_st_y),
                 (arr_ed_x, arr_ed_y))

        vertexs = ((vertex_0_x, vertex_0_y),
                   (vertex_1_x, vertex_1_y),
                   (vertex_2_x, vertex_2_y),
                   (vertex_3_x, vertex_3_y),
                   (vertex_0_x, vertex_0_y))

        wheels = ((wheel_0_x, wheel_0_y),
                  (wheel_1_x, wheel_1_y),
                  (wheel_2_x, wheel_2_y),
                  (wheel_3_x, wheel_3_y))

        return vertexs, arrow, wheels

    def plot(self, ax):
        vertexs, arrow, _ = self.get_vertex()

        i = 0
        self.line_0, = ax.plot([vertexs[i][0], vertexs[i + 1][0]], [vertexs[i][1], vertexs[i + 1][1]], color='green')
        i = 1
        self.line_1, = ax.plot([vertexs[i][0], vertexs[i + 1][0]], [vertexs[i][1], vertexs[i + 1][1]], color='green')
        i = 2
        self.line_2, = ax.plot([vertexs[i][0], vertexs[i + 1][0]], [vertexs[i][1], vertexs[i + 1][1]], color='green')
        i = 3
        self.line_3, = ax.plot([vertexs[i][0], vertexs[i + 1][0]], [vertexs[i][1], vertexs[i + 1][1]], color='green')

        self.arrow = ax.annotate("", xy=arrow[1], xytext=arrow[0], arrowprops=dict(facecolor='black', shrink=0.05))

    def get_turnning_circle_center(self):
        cir_x = 0.0
        cir_y = 0.0
        phi = 0.0
        if self.turn_left > self.turn_right:
            n_x = np.cos(self.theta - np.pi / 2)
            n_y = np.sin(self.theta - np.pi / 2)
            cir_x = self.x + n_x * self.efficient_min_r
            cir_y = self.y + n_y * self.efficient_min_r
            phi = self.theta + np.pi / 2
            phi -= self.reversing_status * self.angular_velocity
        else:
            n_x = np.cos(self.theta + np.pi / 2)
            n_y = np.sin(self.theta + np.pi / 2)
            cir_x = self.x + n_x * self.efficient_min_r
            cir_y = self.y + n_y * self.efficient_min_r
            phi = self.theta - np.pi / 2
            phi += self.reversing_status * self.angular_velocity

        return cir_x, cir_y, phi

    def update(self, frame):
        if self.turn_left == self.turn_right:
            self.x += self.reversing_status * np.cos(self.theta) * self.velocity
            self.y += self.reversing_status * np.sin(self.theta) * self.velocity
        elif self.turn_left > self.turn_right:
            cir_x, cir_y, phi = self.get_turnning_circle_center()
            self.x = cir_x + self.efficient_min_r * np.cos(phi)
            self.y = cir_y + self.efficient_min_r * np.sin(phi)
            self.theta += self.reversing_status * self.angular_velocity
        else:
            cir_x, cir_y, phi = self.get_turnning_circle_center()
            self.x = cir_x + self.efficient_min_r * np.cos(phi)
            self.y = cir_y + self.efficient_min_r * np.sin(phi)
            self.theta -= self.reversing_status * self.angular_velocity

        self.coordinates_update()
        return [self.line_0, self.line_1, self.line_2, self.line_3, self.arrow]

    def coordinates_update(self):
        vertexs, arrow, _ = self.get_vertex()

        i = 0
        self.line_0.set_xdata([vertexs[i][0], vertexs[i + 1][0]])
        self.line_0.set_ydata([vertexs[i][1], vertexs[i + 1][1]])
        i = 1
        self.line_1.set_xdata([vertexs[i][0], vertexs[i + 1][0]])
        self.line_1.set_ydata([vertexs[i][1], vertexs[i + 1][1]])
        i = 2
        self.line_2.set_xdata([vertexs[i][0], vertexs[i + 1][0]])
        self.line_2.set_ydata([vertexs[i][1], vertexs[i + 1][1]])
        i = 3
        self.line_3.set_xdata([vertexs[i][0], vertexs[i + 1][0]])
        self.line_3.set_ydata([vertexs[i][1], vertexs[i + 1][1]])

        self.arrow.xy = arrow[1]
        self.arrow.set_position(arrow[0])

    def get_width(self):
        return self.width

    def get_hight(self):
        return self.hight

    def get_minr(self):
        return self.min_r

    def get_maxc(self):
        return self.max_c

    def get_theta(self):
        return self.theta

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_step_status(self):
        return self.step_status

    def get_efficient_min_r(self):
        return self.efficient_min_r

    def get_reversing_status(self):
        return self.reversing_status

    def set_theta(self, theta):
        self.theta = theta

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_step_status(self, step_status):
        if self.step_status < step_status:
            self.step_status = step_status

    def reset_status(self):
        self.step_status = 0

    def set_velocity(self, velocity):
        self.velocity = velocity
        if self.car_type == 'weight_center':
            self.angular_velocity = velocity / self.min_r
        else:
            self.angular_velocity = velocity / self.rear_min_r

    def set_left_status(self, left_status):
        if left_status != 0 and left_status != 1:
            print('left_status should be 0 or 1')
            return None
        self.turn_left = left_status

    def set_right_status(self, right_status):
        if right_status != 0 and right_status != 1:
            print('right_status should be 0 or 1')
            return None
        self.turn_right = right_status

    def set_reversing_status(self, reversing_status):
        if reversing_status != 1 and reversing_status != -1:
            print('reversing_status should be 1 or -1')
            return None
        self.reversing_status = reversing_status

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

    def plot(self, ax):
        i = 0
        while i < (len(self.space_edge) - 1):
            ax.plot([self.space_edge[i][0], self.space_edge[i + 1][0]], [self.space_edge[i][1], self.space_edge[i + 1][1]], color='black')
            i += 2

