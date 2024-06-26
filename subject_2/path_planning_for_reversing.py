import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import math

from simulation_element import ReverseRelatedMethod, Car, ReverseGarage


class PathPlanningMethod:
    def __init__(self, car, start_status, end_status, garage_type="reverse"):
        self.car = car
        self.garage_type = garage_type
        self.start_status = start_status
        self.end_status = end_status

    def plan_path(self):
        if self.garage_type == "reverse":
            return self.reverse_path_planning()

    def reverse_path_planning(self):
        start_pos = ReverseRelatedMethod.get_position_from_status(self.start_status)
        end_pos = ReverseRelatedMethod.get_position_from_status(self.end_status)

        start_dir = ReverseRelatedMethod.get_direction_from_status(self.start_status)
        end_dir = ReverseRelatedMethod.get_direction_from_status(self.end_status)

        n_start_dir = ReverseRelatedMethod.get_n_from_status(self.start_status)
        n_end_dir = ReverseRelatedMethod.get_n_from_status(self.end_status)

        para_start_pos = ReverseRelatedMethod.get_parallel_pos(start_pos, n_start_dir, self.car.min_r)
        para_end_pos = ReverseRelatedMethod.get_parallel_pos(end_pos, n_end_dir, self.car.min_r)

        para_pos_perpen_foot = np.dot(para_end_pos - para_start_pos, start_dir) * start_dir + para_start_pos

        para_end_to_foot = para_pos_perpen_foot - para_end_pos
        para_end_to_foot = 1. / np.linalg.norm(para_end_to_foot) * para_end_to_foot

        para_end_to_start = para_start_pos - para_end_pos

        para_inter_point = para_end_pos + (np.dot(para_end_to_start, para_end_to_foot) / np.dot(end_dir, para_end_to_foot)) * end_dir # circle's center
        circle_center = para_inter_point

        turn_pos = circle_center - min_r * n_start_dir
        straight_pos = circle_center - min_r * n_end_dir

        return start_pos, turn_pos, straight_pos, end_pos, circle_center

if __name__ == '__main__':
    car_l = 4.6
    car_w = 1.8
    min_r = 3
    m_car = Car(car_l, car_w, min_r)
    m_garage = ReverseGarage(car_l, car_w)

    start_status = [0, m_garage.get_l() + m_garage.get_s() / 2, 170]
    end_status = [m_garage.get_h() + m_garage.get_w() / 2, m_garage.get_l() / 2, 70]

    path_planner = PathPlanningMethod(m_car, start_status, end_status)
    points = path_planner.plan_path()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ReverseRelatedMethod.plot_trajectory(ax, points[0], points[1], points[2], points[3], points[4], min_r)
    m_garage.plot(ax)
    plt.show()
