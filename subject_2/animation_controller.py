import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from path_planning_for_reversing import PathPlanningMethod
from simulation_element import ReverseRelatedMethod, Car, ReverseGarage, ReverseTrajectoryHandler

class ReversingCarAnimationController:
    def __init__(self, car_l, car_w, min_r, start_status, end_status):
        self.car = Car(start_status[2], start_status[0], start_status[1], 'rear_axle_center', car_l, car_w, min_r)
        self.car_last_status = self.car.get_step_status()
        self.garage = ReverseGarage(car_l, car_w)
        self.path_planner = PathPlanningMethod(self.car, start_status, end_status)
        self.end_status = end_status
        self.velocity = 0.05

        self.fig, self.ax = plt.subplots()
        self.trajectory_edge_handler = []
        points = self.path_planner.plan_path()
        vertexs, arrow, wheels = self.car.get_vertex()
        edges, radii = self.path_planner.reversing_path_range(vertexs, arrow, wheels)
        '''
        self.trajectory_handler = ReverseTrajectoryHandler(self.ax, points, points[4], self.car.get_efficient_min_r())
        for i in range(len(edges)):
            self.trajectory_edge_handler.append(ReverseTrajectoryHandler(self.ax, edges[i], points[4], radii[i], 0.3))
        '''

        self.step_1_edges_cal_status = False
        self.step_2_edges_cal_status = False
        self.park_car_successfully = True
        self.restart_flag = False
        self.init_command = False
        self.step_0_command = False
        self.step_01_command = False
        self.step_12_command = False
        self.step_23_command = False
        self.step_3_command = False
        self.step_fail_command = False

        self.is_parking = True

        self.garage.plot(self.ax)
        self.car.plot(self.ax)
        self.fig.canvas.mpl_connect('key_press_event', self.press_key)
        self.fig.canvas.mpl_connect('key_release_event', self.release_key)

        self.step_0_left_edge, self.step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(self.car, self.garage)
        self.step_0_left_point = self.ax.scatter([0], [0], color='blue', alpha=0.0)
        self.step_0_right_point = self.ax.scatter([0], [0], color='red', alpha=0.0)
        if self.step_0_left_edge is not None and self.step_0_right_edge is not None:
            self.step_0_left_point.set_offsets(self.step_0_left_edge)
            self.step_0_right_point.set_offsets(self.step_0_right_edge)
            self.step_0_left_point.set_alpha(1.0)
            self.step_0_right_point.set_alpha(1.0)
        else:
            self.park_car_successfully = False
            print('Coach：当前姿态无法进库，调整位置重新开始')

        self.step_1_upper_point = self.ax.scatter([0], [0], color='blue', alpha=0.0)
        self.step_1_lower_point = self.ax.scatter([0], [0], color='red', alpha=0.0)

        self.step_2_upper_point = self.ax.scatter([0], [0], color='blue', alpha=0.0)
        self.step_2_lower_point = self.ax.scatter([0], [0], color='red', alpha=0.0)

        self.step_1_lower_edge = None
        self.step_1_upper_edge = None
        self.step_2_lower_edge = None
        self.step_2_upper_edge = None

        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Trajectory Plot')

        self.ani = animation.FuncAnimation(self.fig, self.update, frames=np.arange(0, 100), interval=50, blit=True)

    def reset(self):
        self.step_1_edges_cal_status = False
        self.step_2_edges_cal_status = False
        self.park_car_successfully = True
        self.restart_flag = False

        self.init_command = False
        self.step_0_command = False
        self.step_01_command = False
        self.step_12_command = False
        self.step_23_command = False
        self.step_3_command = False
        self.step_fail_command = False
        self.is_parking = True

        self.car.set_step_status(0)
        self.car_last_status = 0

        self.step_0_left_edge, self.step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(self.car, self.garage)
        self.step_0_left_point.set_alpha(0.0)
        self.step_0_right_point.set_alpha(0.0)

        self.step_1_upper_point.set_alpha(0.0)
        self.step_1_lower_point.set_alpha(0.0)

        self.step_2_upper_point.set_alpha(0.0)
        self.step_2_lower_point.set_alpha(0.0)

        if self.step_0_left_edge is not None and self.step_0_right_edge is not None:
            self.step_0_left_point.set_offsets(self.step_0_left_edge)
            self.step_0_right_point.set_offsets(self.step_0_right_edge)
            self.step_0_left_point.set_alpha(1.0)
            self.step_0_right_point.set_alpha(1.0)
        else:
            self.park_car_successfully = False
            return False

        return True

    def show(self):
        plt.show()

    def update_trajectories(self):
        current_status = [self.car.get_x(), self.car.get_y(), np.rad2deg(self.car.get_theta())]
        self.path_planner.set_status(current_status, self.end_status)
        points = self.path_planner.plan_path()
        vertexs, arrow, wheels = self.car.get_vertex()
        edges, radii = self.path_planner.reversing_path_range(vertexs, arrow, wheels)
        self.trajectory_handler.update_status(points, points[4], self.car.get_efficient_min_r())
        for i in range(len(self.trajectory_edge_handler)):
            self.trajectory_edge_handler[i].update_status(edges[i], points[4], radii[i])

    def update(self, frame):
        if self.restart_flag is True:
            if self.reset() is False:
                print('Coach：当前姿态无法进库，调整位置重新开始')
            else:
                self.restart_flag = False
                if self.init_command is False:
                    print('Coach：挂倒档，方向盘回正，直线倒车')
                    self.init_command = True

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.car_last_status == 0 and \
           self.car.get_step_status() == 1:
            if self.car.get_x() < self.step_0_left_edge[0]:
                print('Coach：方向盘打早了，无法进库')
                self.park_car_successfully = False
            elif self.car.get_x() > self.step_0_right_edge[0]:
                print('Coach：方向盘打晚了，无法进库')
                self.park_car_successfully = False

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.car_last_status == 1 and \
           self.car.get_step_status() == 2:
            if self.car.get_y() < self.step_1_lower_edge[1]:
                print('Coach：回正早了，无法进库')
                self.park_car_successfully = False
            elif self.car.get_y() > self.step_1_upper_edge[1]:
                print('Coach：回正晚了，无法进库')
                self.park_car_successfully = False

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.car_last_status == 2 and \
           self.car.get_step_status() == 3:
            if self.car.get_y() < self.step_2_lower_edge[1]:
                print('Coach：停车晚了，进库失败')
                self.park_car_successfully = False
            elif self.car.get_y() > self.step_2_upper_edge[1]:
                print('Coach：停车早了，进库失败')
                self.park_car_successfully = False

        if self.car.get_velocity() < 0.001 and \
           self.is_parking is False and \
           self.car.get_x() < self.car.get_dis_axles() and \
           self.car.get_x() > 0 and \
           self.car.get_y() > self.garage.get_l() and \
           self.car.get_y() < (self.garage.get_l() + self.garage.get_s()) and \
           self.car.get_left_status() == 0 and \
           self.car.get_right_status() == 0:
            self.restart_flag = True
            # print('set restart_flag as True')

        if self.is_parking is True and \
           self.park_car_successfully is False and \
           self.step_fail_command is False:
            print('Coach：停车失败，回到起始位置继续练习')
            self.step_fail_command = True
            self.step_3_command = True
            self.is_parking = False

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.step_01_command is False and \
           self.car_last_status == 0 and \
           self.car.get_x() > self.step_0_left_edge[0] and \
           self.car.get_x() < self.step_0_right_edge[0]:
            print('Coach：方向盘打死')
            self.step_01_command = True

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.step_12_command is False and \
           self.car_last_status == 1 and \
           self.car.get_y() > self.step_1_lower_edge[1] and \
           self.car.get_y() < self.step_1_upper_edge[1]:
            print('Coach：回正')
            self.step_12_command = True

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.step_23_command is False and \
           self.car_last_status == 2 and \
           self.car.get_y() > self.step_2_lower_edge[1] and \
           self.car.get_y() < self.step_2_upper_edge[1]:
            print('Coach：停车')
            self.step_23_command = True

        if self.is_parking is True and \
           self.park_car_successfully is True and \
           self.step_3_command is False and \
           self.car_last_status == 3 and \
           self.car.get_y() > self.step_2_lower_edge[1] and \
           self.car.get_y() < self.step_2_upper_edge[1]:
            print('Coach：成功入库，回到起始位置，继续练习')
            self.step_3_command = True
            self.step_fail_command = True
            self.is_parking = False

        self.car_last_status = self.car.get_step_status()
        plot_handler = []
        plot_handler.extend(self.car.update(frame))
        '''
        self.update_trajectories()
        plot_handler.extend(self.trajectory_handler.update(frame))
        for i in range(len(self.trajectory_edge_handler)):
            plot_handler.extend(self.trajectory_edge_handler[i].update(frame))
        '''

        plot_handler.extend([self.step_0_left_point, self.step_0_right_point])

        if self.park_car_successfully is True and self.car.get_step_status() == 1 and self.step_1_edges_cal_status is False:
            cir_x, cir_y, phi = self.car.get_turnning_circle_center()
            self.step_1_upper_edge, self.step_1_lower_edge = ReverseRelatedMethod.cal_edges_for_step_1(self.car, self.garage, cir_x, cir_y)
            # print('step 1 edges: ', self.step_1_upper_edge, self.step_1_lower_edge)
            self.step_1_edges_cal_status = True
            if self.step_1_upper_edge[0] is not None and self.step_1_lower_edge[0] is not None:
                self.step_1_lower_point.set_offsets(self.step_1_lower_edge)
                self.step_1_upper_point.set_offsets(self.step_1_upper_edge)
                self.step_1_lower_point.set_alpha(1.0)
                self.step_1_upper_point.set_alpha(1.0)
            if self.step_1_upper_edge[0] is not None:
                self.step_1_upper_point.set_offsets(self.step_1_upper_edge)
                self.step_1_upper_point.set_alpha(1.0)
            if self.step_1_lower_edge[0] is not None:
                self.step_1_lower_point.set_offsets(self.step_1_lower_edge)
                self.step_1_lower_point.set_alpha(1.0)

        plot_handler.extend([self.step_1_lower_point, self.step_1_upper_point])

        if self.park_car_successfully is True and self.car.get_step_status() == 2 and self.step_2_edges_cal_status is False:
            self.step_2_upper_edge, self.step_2_lower_edge = ReverseRelatedMethod.cal_edges_for_step_2(self.car, self.garage)
            # print('step 2 edges: ', self.step_2_upper_edge, self.step_2_lower_edge)
            self.step_2_edges_cal_status = True
            self.step_2_upper_point.set_offsets(self.step_2_upper_edge)
            self.step_2_lower_point.set_offsets(self.step_2_lower_edge)
            self.step_2_upper_point.set_alpha(1.0)
            self.step_2_lower_point.set_alpha(1.0)

        plot_handler.extend([self.step_2_lower_point, self.step_2_upper_point])

        return plot_handler

    def press_key(self, event):
        if event.key == 'up':
            self.car.set_reversing_status(1)
            self.car.set_velocity(self.velocity)
        elif event.key == 'down':
            self.car.set_reversing_status(-1)
            self.car.set_velocity(self.velocity)
        elif event.key == 'right':
            if self.car.get_velocity() > 0.0:
                self.car.set_step_status(1)
            self.car.set_right_status(1)
        elif event.key == 'left':
            if self.car.get_velocity() > 0.0:
                self.car.set_step_status(1)
            self.car.set_left_status(1)

    def release_key(self, event):
        if event.key == 'up':
            self.car.set_velocity(0)
        elif event.key == 'down':
            self.car.set_velocity(0)
            if self.car.get_step_status() == 2:
                self.car.set_step_status(3)
        elif event.key == 'right':
            self.car.set_step_status(2)
            self.car.set_right_status(0)
        elif event.key == 'left':
            self.car.set_step_status(2)
            self.car.set_left_status(0)
