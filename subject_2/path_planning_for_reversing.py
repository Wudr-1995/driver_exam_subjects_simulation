import numpy as np
import matplotlib.pyplot as plt
import math

from simulation_element import ReverseRelatedMethod, Car, ReverseGarage


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

    def set_status(self, start_status, end_status):
        self.start_status = start_status
        self.end_status = end_status

    def plan_path(self):
        if self.garage_type == "reverse":
            return self.reversing_path_plan()

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

    def reversing_path_range(self):
        vertexs, arrow, wheels = self.car.get_vertex()
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
