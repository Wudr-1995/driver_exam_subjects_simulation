import numpy as np
import matplotlib.pyplot as plt
import math

# git@github.com:AtsushiSakai/PythonRobotics.git
from ReedsSheppPath import reeds_shepp_path_planning as rs

from simulation_element import ReverseRelatedMethod, Car, ReverseGarage

class Custom_Path:
    def __init__(self):
        self.ctypes = []
        self.positions = []
        self.yaws = []
        self.directions = []

    def append(self, ctype, position, yaw, direction):
        self.ctypes.append(ctype)
        self.positions.append(position)
        self.yaws.append(yaw)
        self.directions.append(direction)

    def to_real_world(self, start_status):
        v_cos = np.cos(-start_status[2])
        v_sin = np.sin(-start_status[2])
        ori_x = start_status[0]
        ori_y = start_status[1]
        theta = start_status[2]
        self.positions = [[np.array([v_cos * pos[0][0] + v_sin * pos[0][1] + ori_x,
                                     -v_sin * pos[0][0] + v_cos * pos[0][1] + ori_y]),
                           np.array([v_cos * pos[1][0] + v_sin * pos[1][1] + ori_x,
                                     -v_sin * pos[1][0] + v_cos * pos[1][1] + ori_y])]
                           for pos in self.positions]
        self.yaws = [rs.pi_2_pi(yaw + theta) for yaw in self.yaws]

    def print(self):
        print(f'ctypes: {self.ctypes}')
        print(f'positions: {self.positions}')
        print(f'yaws: {self.yaws}')
        print(f'directions: {self.directions}')

class PathPlanningMethod:
    def __init__(self, car, start_status, end_status, garage_type="reverse"):
        self.car = car
        self.garage_type = garage_type
        self.start_status = start_status
        self.end_status = end_status

        self.start_pos = None
        self.turn_pos = None
        self.turn_back_pos = None
        self.end_pos = None
        self.circle_center = None
        self.tmp_radius = None

        self.paths = []

    def set_status(self, start_status, end_status):
        self.start_status = start_status
        self.end_status = end_status

    def plan_path(self):
        if self.garage_type == "reverse":
            return self.reversing_path_plan()
        
    def external_reeds_shepp(self):
        step_size = 0.02
        max_curvature = (1. / self.car.get_efficient_min_r())
        tmp_st_status = [self.start_status[0], self.start_status[1], np.deg2rad(self.start_status[2])]
        tmp_ed_status = [self.end_status[0], self.end_status[1], np.deg2rad(self.end_status[2])]
        paths = rs.generate_path(tmp_st_status, tmp_ed_status,
                                 1. / self.car.get_efficient_min_r(), step_size)
        self.paths = []
        for path in paths:
            interpolate_dists_list = rs.calc_interpolate_dists_list(path.lengths,
                                                                    max_curvature * step_size)
            origin_x, origin_y, origin_yaw = 0.0, 0.0, 0.0
            m_path = Custom_Path()
            for (interp_dists, mode, length) in zip(interpolate_dists_list,
                                                    path.ctypes,
                                                    path.lengths):
                x, y, yaw, direction = rs.interpolate(interp_dists[-1], length, mode, max_curvature,
                                                      origin_x, origin_y, origin_yaw)
                m_path.append(mode, [np.array([origin_x, origin_y]), np.array([x, y])], [origin_yaw, yaw], direction)
                origin_x = x
                origin_y = y
                origin_yaw = yaw
            self.paths.append(m_path)

        if len(paths) == 0:
            return None
        best_path_index = paths.index(min(paths, key=lambda p: abs(p.L)))
        self.paths[best_path_index].to_real_world(tmp_st_status)
        return self.paths[best_path_index]

    def reversing_path_plan(self):
        start_pos = ReverseRelatedMethod.get_position_from_status(self.start_status)
        end_pos = ReverseRelatedMethod.get_position_from_status(self.end_status)

        start_dir = ReverseRelatedMethod.get_direction_from_status(self.start_status)
        end_dir = ReverseRelatedMethod.get_direction_from_status(self.end_status)

        n_start_dir = ReverseRelatedMethod.get_n_from_status(self.start_status)
        n_end_dir = ReverseRelatedMethod.get_n_from_status(self.end_status)

        if start_dir[0] > 0:
            n_start_dir = -n_start_dir
            n_end_dir = -n_end_dir

        para_start_pos = ReverseRelatedMethod.get_parallel_pos(start_pos, n_start_dir, self.car.get_efficient_min_r())
        para_end_pos = ReverseRelatedMethod.get_parallel_pos(end_pos, n_end_dir, self.car.get_efficient_min_r())

        para_pos_perpen_foot = np.dot(para_end_pos - para_start_pos, start_dir) * start_dir + para_start_pos

        para_end_to_foot = para_pos_perpen_foot - para_end_pos

        para_end_to_start = para_start_pos - para_end_pos

        para_inter_point = para_end_pos + (np.linalg.norm(para_end_to_foot)**2 / np.dot(end_dir, para_end_to_foot)) * end_dir # circle's center
        circle_center = para_inter_point

        turn_pos = circle_center - self.car.get_efficient_min_r() * n_start_dir
        turn_back_pos = circle_center - self.car.get_efficient_min_r() * n_end_dir

        self.start_pos = start_pos
        self.turn_pos = turn_pos
        self.turn_back_pos = turn_back_pos
        self.end_pos = end_pos
        self.circle_center = circle_center

        return start_pos, turn_pos, turn_back_pos, end_pos, circle_center

    def reversing_path_range(self, vertexs, arrow, wheels):
        # vertexs, arrow, wheels = self.car.get_vertex()
        theta = self.car.get_theta()

        edges = []
        radii = []
        '''
        for i in range(len(vertexs) - 1):
            edges.append(self.cal_reversing_path_edge(vertexs[i], theta))
            radii.append(self.tmp_radius)

        for i in range(len(wheels)):
            edges.append(self.cal_reversing_path_edge(wheels[i], theta))
            radii.append(self.tmp_radius)
            print('radius: ', self.tmp_radius)
            print('vertexs: ', i, vertexs[i])

        '''
        edges.append(self.cal_reversing_path_edge(vertexs[2], theta))
        radii.append(self.tmp_radius)
        edges.append(self.cal_reversing_path_edge(vertexs[3], theta))
        radii.append(self.tmp_radius)
        edges.append(self.cal_reversing_path_edge(wheels[1], theta))
        radii.append(self.tmp_radius)
        # print('radius: ', self.tmp_radius)

        return edges, radii

    def cal_reversing_path_edge(self, vertex, theta):
        dir_vec = np.array([np.cos(theta), np.sin(theta)])
        vertex = np.array(vertex)

        # the second position
        turn_pos = vertex + (self.turn_pos - self.start_pos)
        # turn_pos = vertex - np.linalg.norm(self.turn_pos - self.start_pos) * dir_vec

        center_to_turn_pos = turn_pos - self.circle_center
        delta_theta = np.deg2rad(self.end_status[2] - self.start_status[2])

        # Rotation array
        R = np.array([[np.cos(delta_theta), -np.sin(delta_theta)], \
                      [np.sin(delta_theta), np.cos(delta_theta)]])

        # the third position
        turn_back_pos = R @ center_to_turn_pos + self.circle_center

        # the last position
        end_pos = turn_back_pos + (self.end_pos - self.turn_back_pos)

        self.tmp_radius = np.linalg.norm(center_to_turn_pos)

        return [vertex, turn_pos, turn_back_pos, end_pos]
