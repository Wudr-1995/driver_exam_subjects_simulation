import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from path_planning_for_reversing import PathPlanningMethod
from simulation_element import ReverseRelatedMethod, Car, ReverseGarage, ReverseTrajectoryHandler

car_l = 4.6
car_w = 1.8
min_r = 5
dis_axle = 2.6

# start_status = [8.1, 3.6, 90.0]
# start_status = [1.9943301547061152, 8.384488063516272, np.rad2deg(3.1622005175104473)]
# start_status = [3.2940541198392985, 8.411276390423279, np.rad2deg(3.1622005175104473)]
start_status = [5.0, 8.4, 180]
end_status = [8.1, 1.0, 90]

class ReverseCarAnimationController:
    def __init__(self, car_l, car_w, min_r, start_status, end_status):
        self.car = Car(start_status[2], start_status[0], start_status[1], 'rear_axle_center', car_l, car_w, 2.6, min_r)
        self.garage = ReverseGarage(car_l, car_w)
        self.path_planner = PathPlanningMethod(self.car, start_status, end_status)
        self.end_status = end_status
        self.velocity = 0.2

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

        self.garage.plot(self.ax)
        self.car.plot(self.ax)

        self.fig.canvas.mpl_connect('key_press_event', self.press_key)
        self.fig.canvas.mpl_connect('key_release_event', self.release_key)

        self.step_0_left_point = self.ax.scatter([0], [0], color='blue', alpha=0.0)
        self.step_0_right_point = self.ax.scatter([0], [0], color='red', alpha=0.0)

        self.step_1_upper_point = self.ax.scatter([0], [0], color='green', alpha=0.0)
        self.step_1_lower_point = self.ax.scatter([0], [0], color='yellow', alpha=0.0)

        self.step_2_upper_point = self.ax.scatter([0], [0], color='black', alpha=0.0)
        self.step_2_lower_point = self.ax.scatter([0], [0], color='magenta', alpha=0.0)

        self.step_3_upper_point = self.ax.scatter([0], [0], color='cyan', alpha=0.0)
        self.step_3_lower_point = self.ax.scatter([0], [0], color='orange', alpha=0.0)

        self.step_1_lower_edge = None
        self.step_1_upper_edge = None
        self.step_2_lower_edge = None
        self.step_2_upper_edge = None

        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Trajectory Plot')

        self.ctype = None
        self.direction = None

        self.ani = animation.FuncAnimation(self.fig, self.update, frames=np.arange(0, 100), interval=50, blit=True)

    def fondamental_layer(self):
        cir_x, cir_y, phi = self.car.get_turnning_circle_center()

        self.step_0_left_edge, self.step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(self.car, self.garage)
        self.step_1_upper_edge, self.step_1_lower_edge = ReverseRelatedMethod.cal_edges_for_step_1(self.car, self.garage, cir_x, cir_y)
        self.step_2_upper_edge, self.step_2_lower_edge = ReverseRelatedMethod.cal_edges_for_step_2(self.car, self.garage)
        self.step_3_upper_edge, self.step_3_lower_edge = ReverseRelatedMethod.cal_edges_for_step_3(self.car, self.garage)

        m_path = self.path_planner.external_reeds_shepp()
        self.operation = None
        self.direction = None
        if m_path is not None:
            for positions, ctype, direction in zip(m_path.positions, m_path.ctypes, m_path.directions):
                dis = np.linalg.norm(positions[0] - positions[1])
                if dis > 0.1:
                    self.operation = ctype
                    self.direction = direction
                    break

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
        plot_handler = []
        plot_handler.extend(self.car.update(frame))
        '''
        self.update_trajectories()
        plot_handler.extend(self.trajectory_handler.update(frame))
        for i in range(len(self.trajectory_edge_handler)):
            plot_handler.extend(self.trajectory_edge_handler[i].update(frame))
        '''

        plot_handler.extend([self.step_0_left_point, self.step_0_right_point])

        self.step_0_left_edge, self.step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(self.car, self.garage)
        if self.step_0_left_edge is not None and self.step_0_right_edge is not None:
            self.step_0_left_point.set_offsets(self.step_0_left_edge)
            self.step_0_right_point.set_offsets(self.step_0_right_edge)
            print('step 0 edges: ', self.step_0_left_edge, self.step_0_right_edge)
            self.step_0_left_point.set_alpha(1.0)
            self.step_0_right_point.set_alpha(1.0)
        else:
            print('no efficient step 0 edges')
            self.step_0_left_point.set_alpha(0.0)
            self.step_0_right_point.set_alpha(0.0)

        cir_x, cir_y, phi = self.car.get_turnning_circle_center()
        self.step_1_upper_edge, self.step_1_lower_edge = ReverseRelatedMethod.cal_edges_for_step_1(self.car, self.garage, cir_x, cir_y)
        if self.step_1_upper_edge is not None and self.step_1_lower_edge is not None:
            self.step_1_lower_point.set_offsets(self.step_1_lower_edge)
            self.step_1_upper_point.set_offsets(self.step_1_upper_edge)
            print('step 1 edges: ', self.step_1_lower_edge, self.step_1_upper_edge)
            self.step_1_lower_point.set_alpha(1.0)
            self.step_1_upper_point.set_alpha(1.0)
        else:
            print('no efficient step 1 edges')
            self.step_1_lower_point.set_alpha(0.0)
            self.step_1_upper_point.set_alpha(0.0)

        plot_handler.extend([self.step_1_lower_point, self.step_1_upper_point])

        self.step_2_upper_edge, self.step_2_lower_edge = ReverseRelatedMethod.cal_edges_for_step_2(self.car, self.garage)
        # print('step 2 edges: ', self.step_2_upper_edge, self.step_2_lower_edge)
        if self.step_2_upper_edge is not None and self.step_2_lower_edge is not None:
            self.step_2_upper_point.set_offsets(self.step_2_upper_edge)
            self.step_2_lower_point.set_offsets(self.step_2_lower_edge)
            print('step 2 edges: ', self.step_2_lower_edge, self.step_2_upper_edge)
            self.step_2_upper_point.set_alpha(1.0)
            self.step_2_lower_point.set_alpha(1.0)
        else:
            print('no efficient step 2 edges')
            self.step_2_upper_point.set_alpha(0.0)
            self.step_2_lower_point.set_alpha(0.0)

        plot_handler.extend([self.step_2_lower_point, self.step_2_upper_point])

        self.step_3_upper_edge, self.step_3_lower_edge = ReverseRelatedMethod.cal_edges_for_step_3(self.car, self.garage)

        if self.step_3_lower_edge is not None and self.step_3_upper_edge is not None:
            self.step_3_lower_point.set_offsets(self.step_3_lower_edge)
            self.step_3_upper_point.set_offsets(self.step_3_upper_edge)
            print('step 3 edges: ', self.step_3_lower_edge, self.step_3_upper_edge)
            self.step_3_lower_point.set_alpha(1.0)
            self.step_3_upper_point.set_alpha(1.0)
        else:
            print('no efficient step 3 edges')
            self.step_3_lower_point.set_alpha(0.0)
            self.step_3_upper_point.set_alpha(0.0)

        plot_handler.extend([self.step_3_lower_point, self.step_3_upper_point])

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
        elif event.key == 'right':
            self.car.set_step_status(2)
            self.car.set_right_status(0)
        elif event.key == 'left':
            self.car.set_step_status(2)
            self.car.set_left_status(0)

m_controller = ReverseCarAnimationController(car_l, car_w, min_r, start_status, end_status)
m_controller.show()

# m_car = Car(start_status[2], start_status[0], start_status[1], 'rear_axle_center', car_l, car_w, dis_axle, min_r)
# m_garage = ReverseGarage(car_l, car_w)
# m_path_planner = PathPlanningMethod(m_car, start_status, end_status)
# 
# m_path = m_path_planner.external_reeds_shepp()
# m_path.print()
# 
# """
# points = m_path_planner.plan_path()
# vertexs, arrow, wheels = m_car.get_vertex()
# edges, radii = m_path_planner.reversing_path_range(vertexs, arrow, wheels)
# trajectory_edge_handler = []
# """
# 
# fig, ax = plt.subplots()
# ax.grid(True)
# m_garage.plot(ax)
# m_car.plot(ax)
# 
# """
# trajectory_handler = ReverseTrajectoryHandler(ax, points, points[4], m_car.get_efficient_min_r())
# for i in range(len(edges)):
#     trajectory_edge_handler.append(ReverseTrajectoryHandler(ax, edges[i], points[4], radii[i], 0.3))
# """
# 
# step_0_left_edge, step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(m_car, m_garage, left=True)
# if step_0_left_edge is not None:
#     print('The edges of step 0 are legal ', step_0_left_edge, step_0_right_edge)
#     ax.scatter([step_0_left_edge[0]], [step_0_left_edge[1]], color='blue')
#     ax.scatter([step_0_right_edge[0]], [step_0_right_edge[1]], color='red')
# else:
#     print('step 0 edges are illegal')
# 
# cir_x, cir_y, phi = m_car.get_turnning_circle_center(left=True)
# _, _, phi = m_car.get_current_center(left=True)
# print('phi: ', phi)
# step_1_upper_edge, step_1_lower_edge = ReverseRelatedMethod.cal_edges_for_step_1(m_car, m_garage, cir_x, cir_y)
# if step_1_lower_edge is not None:
#     print('The edges of step 1 are legal ', step_1_upper_edge, step_1_lower_edge)
#     ax.scatter([step_1_upper_edge[0]], [step_1_upper_edge[1]], color='blue')
#     ax.scatter([step_1_lower_edge[0]], [step_1_lower_edge[1]], color='red')
# else:
#     print('step 1 edges are illegal')
# 
# step_3_upper_edge, step_3_lower_edge = ReverseRelatedMethod.cal_edges_for_step_3(m_car, m_garage, left=True)
# if step_3_lower_edge is not None:
#     print('The edges of step 3 are legal ', step_3_upper_edge, step_3_lower_edge)
#     # ax.scatter([step_3_upper_edge[0]], [step_3_upper_edge[1]], color='blue')
#     # ax.scatter([step_3_lower_edge[0]], [step_3_lower_edge[1]], color='red')
# else:
#     print('step 3 edges are illegal')
# 
# print('In the garage? ', ReverseRelatedMethod.is_car_in_parking_slot(m_car, m_garage))
# print('In the initial range? ', ReverseRelatedMethod.is_car_in_returning_slot(m_car, m_garage))
# print('Intersections? ', ReverseRelatedMethod.check_intersections(m_car, m_garage))
# print('Can the car reach the initial range straight? ', ReverseRelatedMethod.can_car_reach_initial_position_straight(m_car, m_garage, left=True))
# # print('Can the car get into the garage? ', ReverseRelatedMethod.can_car_get_into_parking_slot(m_car, m_garage))
# 
# plt.show()
