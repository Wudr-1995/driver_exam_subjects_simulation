import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import fsolve

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
    step_0_edge_beta = None
    step_0_upper_edge_beta = None
    step_0_left_edge = None
    step_0_right_edge = None
    step_1_upper_edge = None
    step_1_lower_edge = None
    step_3_upper_edge = None
    step_3_lower_edge = None

    def __init__(self):
        pass

    @staticmethod
    def plot_trajectory(ax, points, center, radius, alpha=1.0):
        # get start angle of the curve
        start_angle = np.arctan2(points[1][1] - center[1], points[1][0] - center[0])
        end_angle = np.arctan2(points[2][1] - center[1], points[2][0] - center[0])

        if np.abs(start_angle - end_angle) > np.pi:
            if start_angle < 0.0:
                start_angle += 2 * np.pi
            if end_angle < 0.0:
                end_angle += 2 * np.pi

        # make sure the direction of the angles
        if start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle

        angles = np.linspace(start_angle, end_angle, 10)
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

    @staticmethod
    def on_segment(p, q, r):
        """
        Check if point q lies on line segment pr
        """
        if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
            return True
        return False

    @staticmethod
    def orientation(p, q, r):
        """
        Calculate the orientation of the triplet (p, q, r)
        Returns:
        0 -> p, q and r are collinear
        1 -> Clockwise
        2 -> Counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        elif val > 0:
            return 1
        else:
            return 2

    @staticmethod
    def do_intersect(p1, q1, p2, q2):
        """
        Determine if line segments p1q1 and p2q2 intersect
        """
        # Find the four orientations needed for general and special cases
        o1 = ReverseRelatedMethod.orientation(p1, q1, p2)
        o2 = ReverseRelatedMethod.orientation(p1, q1, q2)
        o3 = ReverseRelatedMethod.orientation(p2, q2, p1)
        o4 = ReverseRelatedMethod.orientation(p2, q2, q1)

        # General case
        if o1 != o2 and o3 != o4:
            return True

        # Special cases
        # p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if o1 == 0 and ReverseRelatedMethod.on_segment(p1, p2, q1):
            return True

        # p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if o2 == 0 and ReverseRelatedMethod.on_segment(p1, q2, q1):
            return True

        # p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if o3 == 0 and ReverseRelatedMethod.on_segment(p2, p1, q2):
            return True

        # p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if o4 == 0 and ReverseRelatedMethod.on_segment(p2, q1, q2):
            return True

        # Doesn't intersect
        return False

    @staticmethod
    def check_intersections(car, garage):
        vector_list1, _, _ = car.get_vertex()
        vector_list2 = garage.space_edge
        """
        Check if any line segment formed by consecutive pairs of points in vector_list1
        intersects with any line segment formed by consecutive pairs of points in vector_list2.

        Parameters:
        vector_list1: List of tuples representing points [(x0, y0), (x1, y1), (x2, y2), ...]
        vector_list2: List of tuples representing points [(x0, y0), (x1, y1), (x2, y2), ...]

        Returns:
        True if any segments intersect, False otherwise.
        """
        for i in range(0, len(vector_list1) - 1, 2):
            for j in range(0, len(vector_list2) - 1, 2):
                p1, q1 = vector_list1[i], vector_list1[i + 1]
                p2, q2 = vector_list2[j], vector_list2[j + 1]
                if ReverseRelatedMethod.do_intersect(p1, q1, p2, q2):
                    return True
        return False

    @staticmethod
    def cal_fixed_value_for_step0_edges(car, garage):
        def equation_for_edge_step_0(beta):
            return car.get_width() / np.cos(beta) + car.get_length() * np.sin(beta) - garage.get_w()
        initial_beta = 0.1
        solution, info, ier, msg = fsolve(equation_for_edge_step_0, initial_beta, full_output=True)
        if ier == 1:
            ReverseRelatedMethod.step_0_edge_beta = solution
        # else:
        #     print("Solution did not converge:", msg)

    @staticmethod
    def cal_fixed_value_for_step1_upper_edge(car, garage, circle_center_x, circle_center_y):
        def equation_for_edge_step_1_upper_edge(beta):
            return (garage.get_h() + garage.get_w() - circle_center_x) * np.cos(beta) - \
                   car.get_length() * np.sin(beta) * np.cos(beta) - \
                   (circle_center_y - garage.get_l()) * np.sin(beta) - \
                   (car.get_efficient_min_r() + car.get_width() / 2)
        initial_beta = 0.01
        solution, info, ier, msg = fsolve(equation_for_edge_step_1_upper_edge, initial_beta, full_output=True)
        if ier == 1:
            ReverseRelatedMethod.step_1_upper_edge = solution
        # else:
        #     print("Solution of upper edeg did not converge:", msg)
        # print('upper edge: ', solution)
        return ier

    @staticmethod
    def cal_fixed_value_for_step1_lower_edge(car, garage, circle_center_x, circle_center_y):
        def equation_for_edge_step_1_lower_edge(beta):
            '''
            item0 = -2 * (car.get_efficient_min_r() - car.get_width() / 2) * np.cos(beta) * np.cos(beta)
            item1 = (garage.get_h() + circle_center_x) * np.cos(beta)
            item2 = car.get_length() * np.sin(beta) * np.cos(beta)
            item3 = (circle_center_y - garage.get_l()) * np.sin(beta)
            item4 = (car.get_efficient_min_r() + car.get_width() / 2) ** 2
            return item0 + item1 + item2 + item3 + item4
            '''
            return - circle_center_y * np.sin(beta) + car.get_efficient_min_r() * np.sin(beta)**2 - \
            (circle_center_x - garage.get_h()) * np.cos(beta) - car.get_efficient_min_r() * np.cos(beta)**2 + \
            0.5 * car.get_width() - car.get_length() * np.sin(beta) * np.cos(beta) + garage.get_l() * np.sin(beta)
        initial_beta = 0.01
        solution, info, ier, msg = fsolve(equation_for_edge_step_1_lower_edge, initial_beta, full_output=True)
        if ier == 1:
            ReverseRelatedMethod.step_1_lower_edge = solution
        # else:
        #     print("Solution of lower edge did not converge:", msg)
        # print('lower edge: ', solution)
        return ier

    @staticmethod
    def mirror_transform(x, y, theta, garage, left=True):
        right_edge = garage.get_h() * 2 + garage.get_w()
        if left:
            return x, y, theta
        else:
            theta = np.pi - theta
            theta = theta if theta < np.pi else (theta - 2 * np.pi)
            return right_edge - x, y, theta


    @staticmethod
    def cal_edges_for_step_0(car, garage, left=True):
        # only take 0.5*pi of theta at the end position into account
        ## calcualte the outside edge, for the left-reversing car, it is the right edge
        car_x = car.get_x()
        car_y = car.get_y()
        car_theta = car.get_theta()
        car_x, car_y, car_theta = ReverseRelatedMethod.mirror_transform(car_x, car_y, car_theta, garage, left)
        left_edge, right_edge = None, None
        r = car.get_efficient_min_r() + car.get_width() / 2
        tail_length = (car.get_length() - car.get_dis_axles()) / 2
        R = np.sqrt(r ** 2 + tail_length ** 2)
        H = garage.get_h()
        L = garage.get_l()
        W = garage.get_w()
        S = garage.get_s()

        # if np.abs(np.rad2deg(car_theta - np.pi / 2.0)) < 0.01:
        #     return None, None

        left_ver = np.array([H, L]) # the left vertex
        right_ver = np.array([H + W, L]) # the right vertex
        start_pos = np.array([car_x, car_y])
        start_dir = np.array([np.cos(car_theta), np.sin(car_theta)])

        n_start_dir = np.array([-start_dir[1], start_dir[0]])
        # if start_dir[0] > 0:
        #     n_start_dir = -n_start_dir

        para_start_pos = ReverseRelatedMethod.get_parallel_pos(start_pos, n_start_dir, car.get_efficient_min_r())

        circle_center_x = para_start_pos[0]
        delta_x, delta_y = 0.0, 0.0
        if np.abs(np.rad2deg(car_theta - np.pi / 2.0)) > 0.01:
            circle_center_x = right_ver[0] - r
            delta_x = circle_center_x - para_start_pos[0]
            delta_y = delta_x * start_dir[1] / start_dir[0]
        circle_center_y = para_start_pos[1] + delta_y
        circle_center = np.array([circle_center_x, circle_center_y])

        if circle_center_y < L and np.abs(np.rad2deg(car_theta - np.pi / 2.0)) > 0.01:
            delta_x, delta_y = 0.0, 0.0
            circle_center_x = right_ver[0] - R
            delta_x = circle_center_x - para_start_pos[0]
            delta_y = delta_x * start_dir[1] / start_dir[0]
            circle_center_y = para_start_pos[1] + delta_y
            circle_center = np.array([circle_center_x, circle_center_y])
        elif circle_center_y - L < tail_length and np.abs(np.rad2deg(car_theta - np.pi / 2.0)) > 0.01:
            perp_foot = para_start_pos + ((right_ver - para_start_pos) @ start_dir) * start_dir
            circle_center = perp_foot + start_dir * np.sqrt(R ** 2 - np.linalg.norm(right_ver - perp_foot) ** 2)

        right_edge = circle_center - car.get_efficient_min_r() * n_start_dir

        ReverseRelatedMethod.step_0_left_edge = None
        ReverseRelatedMethod.step_0_right_edge = None

        r = car.get_efficient_min_r() - car.get_width() / 2
        circle_center_x = H - r
        delta_x, delta_y = 0.0, 0.0
        if np.abs(np.rad2deg(car_theta - np.pi / 2.0)) > 0.01:
            delta_x = circle_center_x - para_start_pos[0]
            delta_y = delta_x * start_dir[1] / start_dir[0]
        else:
            return None, None
        circle_center_y = para_start_pos[1] + delta_y
        circle_center = np.array([circle_center_x, circle_center_y])

        if circle_center_y < L:
            perp_foot = para_start_pos + ((left_ver - para_start_pos) @ start_dir) * start_dir
            if r > np.linalg.norm(left_ver - perp_foot):
                circle_center = perp_foot + start_dir * np.sqrt(r ** 2 - np.linalg.norm(left_ver - perp_foot) ** 2)
            else:
                return None, None

        left_edge = circle_center - car.get_efficient_min_r() * n_start_dir

        single_edge = None
        r = car.get_efficient_min_r() + car.get_width() / 2
        tmp_R = np.sqrt(r ** 2 + (car.get_length() - tail_length) ** 2)
        D = S + L - start_pos[1] + car.get_efficient_min_r()
        dy = D - tmp_R

        if np.abs(start_dir[1]) < 0.001 and dy < 0:
            return None, None
        elif -1 * start_dir[1] > 0.001:
            if dy <= 0.0:
                return None, None
            single_edge = np.array([start_pos[0] + dy / np.tan(car_theta), start_pos[1] + dy])
            if single_edge[0] <= left_edge[0]:
                return None, None
            if single_edge[0] < right_edge[0]:
                right_edge = single_edge
        elif -1 * start_dir[1] < -0.001:
            single_edge = np.array([start_pos[0] + dy / np.tan(car_theta), start_pos[1] + dy])
            if single_edge[0] > right_edge[0]:
                return None, None
            if single_edge[0] > left_edge[0]:
                left_edge = single_edge

        if left_edge[0] is not None:
            left_edge_x, left_edge_y, _ = ReverseRelatedMethod.mirror_transform(left_edge[0], left_edge[1], 0, garage, left)
            left_edge[0] = left_edge_x
            left_edge[1] = left_edge_y

        if right_edge[0] is not None:
            right_edge_x, right_edge_y, _ = ReverseRelatedMethod.mirror_transform(right_edge[0], right_edge[1], 0, garage, left)
            right_edge[0] = right_edge_x
            right_edge[1] = right_edge_y

        if left is False:
            tmp = left_edge
            left_edge = right_edge
            right_edge = tmp

        if left_edge[0] > right_edge[0]:
            return None, None

        ReverseRelatedMethod.step_0_left_edge = left_edge
        ReverseRelatedMethod.step_0_right_edge = right_edge

        return left_edge, right_edge

        '''
        # take the different theta at the end position into account
        largest_r = np.sqrt(np.power((car.get_length() + car.get_dis_axles()) / 2) + np.power(car.get_minr()))
        d = (car.get_y() - car.get_minr() * np.cos(car.get_theta()) + largest_r - garage.get_s() - garage.get_l()) / np.sin(car.get_theta())
        if ReverseRelatedMethod.step_0_edge_beta == None:
            ReverseRelatedMethod.cal_fixed_value_for_step0_edges(car, garage)
        L = garage.get_l()
        H = garage.get_h()
        w = car.get_width()
        l = car.get_length()
        d_ax = car.get_dis_axles()
        beta = ReverseRelatedMethod.step_0_edge_beta

        start_pos = np.array([car.get_x(), car.get_y()])
        end_pos = np.array([0.5 * w / np.cos(beta) + H + (0.2 * w * np.tan(beta) + (l + d_ax) / 2) * np.sin(beta), \
                      L - (0.5 * w * np.tan(beta) + (l + d_ax) / 2) * np.cos(beta)])

        start_dir = np.array([np.cos(car.get_theta()), np.sin(car.get_theta())])
        end_dir = np.array([np.cos(beta + np.pi / 2), np.sin(beta + np.pi / 2)])

        turn_pos_0 = cal_turn_pos(car, start_pos, start_dir, end_pos, end_dir)

        end_pos = np.array([H + W - 0.5 * w / np.cos(beta) - (0.2 * w * np.tan(beta) + (l + d_ax) / 2) * np.sin(beta), \
                      L - (0.5 * w * np.tan(beta) + (l + d_ax) / 2) * np.cos(beta)])
        end_dir = np.array([np.cos(np.pi / 2 - beta), np.sin(np.pi / 2 - beta)])

        turn_pos_1 = cal_turn_pos(car, start_pos, start_dir, end_pos, end_dir)

        tmp_turn_pos = start_pos - start_dir * d

        theta = car.get_theta()

        if theta > np.pi / 2:
            if theta > np.pi and theta < 1.5 * np.pi:
                if tmp_turn_pos[1] < turn_pos_0[1]:
                    return None, None
                turn_pos_1 = tmp_turn_pos if tmp_turn_pos[1] < turn_pos_1[1] else turn_pos_1
            else:
                if tmp_turn_pos[1] > turn_pos_1[1]:
                    return None, None
                turn_pos_0 = tmp_turn_pos if tmp_turn_pos[1] > turn_pos_0[1] else turn_pos_0
        else:
            if theta < 0 and theta > -0.5 * np.pi:
                if tmp_turn_pos[1] > turn_pos_1[1]:
                    return None, None
                turn_pos_0 = tmp_turn_pos if tmp_turn_pos[1] > turn_pos_0[1] else turn_pos_0
            else:
                if tmp_turn_pos[1] < turn_pos_0[1]:
                    return None, None
                turn_pos_1 = tmp_turn_pos if tmp_turn_pos[1] < turn_pos_1[1] else turn_pos_1

        return turn_pos_0, turn_pos_1
        '''

    @staticmethod
    def cal_edges_for_step_1(car, garage, circle_center_x, circle_center_y):
        upper_edge, lower_edge = None, None
        r = car.get_efficient_min_r() - car.get_width() / 2
        tail_length = (car.get_length() - car.get_dis_axles()) / 2
        R = np.sqrt(r ** 2 + tail_length ** 2)
        H = garage.get_h()
        L = garage.get_l()
        W = garage.get_w()

        r = car.get_efficient_min_r() + car.get_width() / 2
        garage_right_vertex = np.array([H + W - 0.08, L])
        R = np.linalg.norm(garage_right_vertex - np.array([circle_center_x, circle_center_y]))
        other_lower_edge = np.arctan2((L - circle_center_y), (H + W - circle_center_x))
        if np.abs(r / R) > 1.0:
            other_lower_edge = None
        else:
            other_lower_edge -= np.arccos(r / R)

        r = car.get_efficient_min_r() - car.get_width() / 2
        garage_left_vertex = np.array([H + 0.08, L])
        R = np.linalg.norm(garage_left_vertex - np.array([circle_center_x, circle_center_y]))
        other_upper_edge = 0
        if r < R:
            other_upper_edge = None

        ier0 = ReverseRelatedMethod.cal_fixed_value_for_step1_upper_edge(car, garage, circle_center_x, circle_center_y)
        ier1 = ReverseRelatedMethod.cal_fixed_value_for_step1_lower_edge(car, garage, circle_center_x, circle_center_y)

        if ier0 == 1:
            upper_edge = np.array([circle_center_x + car.get_efficient_min_r() * np.cos(ReverseRelatedMethod.step_1_upper_edge[0]), \
                                   circle_center_y + car.get_efficient_min_r() * np.sin(ReverseRelatedMethod.step_1_upper_edge[0])])
        if ier1 == 1:
            if other_lower_edge is not None and ReverseRelatedMethod.step_1_lower_edge[0] < other_lower_edge:
                ReverseRelatedMethod.step_1_lower_edge[0] = other_lower_edge
            lower_edge = np.array([circle_center_x + car.get_efficient_min_r() * np.cos(ReverseRelatedMethod.step_1_lower_edge[0]), \
                                   circle_center_y + car.get_efficient_min_r() * np.sin(ReverseRelatedMethod.step_1_lower_edge[0])])

        # if other_lower_edge is None or other_upper_edge is None:
        #     return None, None

        # if lower_edge is None or upper_edge is None:
        #     return None, None

        if (upper_edge is None) or \
           (lower_edge is None) or \
           (upper_edge[1] < lower_edge[1]):
            return None, None

        '''
        print('ier0, ier1: ', ier0, ier1)
        print('upper edge: ', ReverseRelatedMethod.step_1_upper_edge)
        print('edges in method: ', upper_edge, lower_edge)
        '''
        return upper_edge, lower_edge

    @staticmethod
    def cal_edges_for_step_2(car, garage):
        v, a, w = car.get_vertex()
        d0 = 0.

        if v[0][1] > garage.get_l():
            d0 = v[0][1] - garage.get_l()

        if v[3][1] > garage.get_l() and v[3][1] > v[0][1]:
            d0 = v[3][1] - garage.get_l()

        rest_distance_0 = d0 / np.sin(car.get_theta())

        d1 = 0.

        h = garage.get_h()
        w = garage.get_w()
        if car.get_theta() < 0.5 * np.pi:
            d1_1 = v[1][0] - h
            d1_2 = v[2][0] - h
            d1 = d1_1 if d1_1 < d1_2 else d1_2

        if car.get_theta() > 0.5 * np.pi:
            d1_1 = (h + w) - v[1][0]
            d1_2 = (h + w) - v[2][0]
            d1 = d1_1 if d1_1 < d1_2 else d1_2

        rest_distance_1 = d1 / np.abs(np.cos(car.get_theta()))

        car_dir = np.array([np.cos(car.get_theta()), np.sin(car.get_theta())])
        car_pos = np.array([car.get_x(), car.get_y()])

        tail_length = (car.get_length() - car.get_dis_axles()) / 2 + 0.2
        upper_edge_point = car_pos - rest_distance_0 * car_dir
        lower_edge_point = car_pos - rest_distance_1 * car_dir
        if lower_edge_point[1] < 0.0:
            lower_edge_point += np.array([(tail_length - lower_edge_point[1]) * car_dir[0] / car_dir[1], tail_length - lower_edge_point[1]])

        return upper_edge_point, lower_edge_point

    @staticmethod
    def cal_edges_for_step_3(car, garage, left=True):
        car_x = car.get_x()
        car_y = car.get_y()
        car_theta = car.get_theta()
        car_x, car_y, car_theta = ReverseRelatedMethod.mirror_transform(car_x, car_y, car_theta, garage, left)

        eff_min_r = car.get_efficient_min_r()
        tail_len = (car.get_length() - car.get_dis_axles()) / 2
        r = eff_min_r + car.get_width() * 0.5

        R = np.sqrt(r ** 2 + tail_len ** 2)
        r = eff_min_r - car.get_width() * 0.5

        H = garage.get_h()
        L = garage.get_l()
        W = garage.get_w()
        S = garage.get_s()

        left_ver = np.array([H, L]) # the left vertex
        right_ver = np.array([H + W, L]) # the right vertex
        cur_pos = np.array([car_x, car_y])
        cur_dir = np.array([np.cos(car_theta), np.sin(car_theta)])

        n_cur_dir = np.array([-cur_dir[1], cur_dir[0]])

        left_edge, right_edge = None, None

        para_cur_pos = ReverseRelatedMethod.get_parallel_pos(cur_pos, n_cur_dir, eff_min_r)
        para_cur_to_left_ver = left_ver - para_cur_pos
        proj_len = np.dot(para_cur_to_left_ver, cur_dir)
        perpen_foot = proj_len * cur_dir + para_cur_pos
        min_dis = np.linalg.norm(perpen_foot - left_ver)
        shift_dis = proj_len - np.sqrt(r ** 2 - min_dis ** 2) if r > min_dis else None
        if shift_dis is not None:
            left_edge = cur_pos + shift_dis * cur_dir # lower edge from the inner side

        # para_cur_pos = ReverseRelatedMethod.get_parallel_pos(cur_pos, n_cur_dir, eff_min_r)
        dis_to_right_edge = right_ver[0] - para_cur_pos[0]
        delta_x = dis_to_right_edge - R if np.abs(car_theta - np.pi / 2) > 0.001 else 0.0
        delta_y = delta_x * (cur_dir[1] / cur_dir[0]) if np.abs(car_theta - np.pi / 2) > 0.001 else 0.0
        right_edge = cur_pos + np.array([delta_x, delta_y])

        upper_edge, lower_edge = None, None
        ReverseRelatedMethod.step_3_upper_edge = upper_edge
        ReverseRelatedMethod.step_3_lower_edge = lower_edge
        if left_edge is None or right_edge is None:
            return None, None
        # print('in the function, check point 2: ', left_edge, right_edge, right_ver[0] - para_cur_pos[0], R, delta_x, delta_y, car_theta)
        R = eff_min_r + car.get_width() * 0.5
        R = np.sqrt(R ** 2 + (car.get_length() - tail_len) ** 2)
        if np.abs(car_theta - np.pi / 2) > 0.001:
            # para_cur_pos = ReverseRelatedMethod.get_parallel_pos(cur_pos, n_cur_dir, eff_min_r)
            D = S + L - para_cur_pos[1]
            delta_y = D - R
            delta_x = delta_y / (cur_dir[1] / cur_dir[0])
            upper_edge = cur_pos + np.array([delta_x, delta_y])
        else:
            upper_edge = np.array([cur_pos[0], S + L - R])

        # print('in the function, check point 2: ', left_edge, right_edge, right_ver[0] - para_cur_pos[0], R, delta_x, delta_y, car_theta, upper_edge)

        if cur_dir[0] - 0.0 > 0.001:
            if right_edge[1] < upper_edge[1]:
                upper_edge = right_edge
            lower_edge = left_edge
        else:
            lower_edge = left_edge if left_edge[1] > right_edge[1] else right_edge

        # print('in the function, check point 1: ', upper_edge, lower_edge)

        ReverseRelatedMethod.step_3_upper_edge = upper_edge
        ReverseRelatedMethod.step_3_lower_edge = lower_edge
        if lower_edge[1] > upper_edge[1]:
            return None, None

        ReverseRelatedMethod.step_3_upper_edge = upper_edge
        ReverseRelatedMethod.step_3_lower_edge = lower_edge

        return upper_edge, lower_edge

    @staticmethod
    def is_car_in_parking_slot(car, garage):
        c_vertex, _, _ = car.get_vertex()
        g_v = garage.get_vertex()
        def cross_product(v1, v2):
            return np.cross(v1, v2)

        def vector(p1, p2):
            return np.array(p2) - np.array(p1)

        # Calculate the vectors for the four edges of the rectangle
        V1 = vector(g_v[0], g_v[1])
        V2 = vector(g_v[1], g_v[2])
        V3 = vector(g_v[2], g_v[3])
        V4 = vector(g_v[3], g_v[0])

        result = True
        for p in c_vertex:
            # Calculate the vectors from the rectangle vertices to the point
            VP1 = vector(g_v[0], p)
            VP2 = vector(g_v[1], p)
            VP3 = vector(g_v[2], p)
            VP4 = vector(g_v[3], p)

            # Calculate the cross products for each pair of vectors
            C1 = cross_product(V1, VP1)
            C2 = cross_product(V2, VP2)
            C3 = cross_product(V3, VP3)
            C4 = cross_product(V4, VP4)

            # Check the signs of the cross products
            if not((C1 > 0 and C2 > 0 and C3 > 0 and C4 > 0) or \
               (C1 < 0 and C2 < 0 and C3 < 0 and C4 < 0)):
                result = False

        return result

    @staticmethod
    def is_car_in_returning_slot(car, garage):
        theta = car.get_theta()
        wheelbase = car.get_dis_axles()
        car_dir = np.array([np.cos(theta), np.sin(theta)])
        rear_pos = np.array([car.get_x(), car.get_y()])
        front_pos = rear_pos + wheelbase * car_dir
        edges = garage.get_init_edge()
        result = 0
        if (front_pos[0] < edges[0][0]) and \
           (rear_pos[0] > edges[0][0]) and \
           (rear_pos[1] > edges[0][1]) and \
           (rear_pos[1] < edges[1][1]):
            result = 1

        if (front_pos[0] > edges[2][0]) and \
           (rear_pos[0] < edges[2][0]) and \
           (rear_pos[1] > edges[2][1]) and \
           (rear_pos[1] < edges[3][1]):
            result = 2

        # 0: not in
        # 1: in the left initial range
        # 2: in the right initial range
        return result

    @staticmethod
    def can_car_reach_initial_position_straight(car, garage, left=True):
        theta = car.get_theta()
        car_pos = np.array([car.get_x(), car.get_y()])
        edges = garage.get_init_edge()
        edge = None
        if left:
            edge = edges[:2]
        else:
            edge = edges[2:]
        dis_to_edge = car_pos[0] - edge[0][0]
        y_proj = car_pos[1] - dis_to_edge * np.tan(theta)
        result = False

        if (y_proj > edge[0][1]) and \
           (y_proj < edge[1][1]):
            result = True

        # print('checking ReverseRelatedMethod.can_car_reach_initial_position_straight ', edge[0], edge[1], y_proj, dis_to_edge, car_pos, result)

        return result

    """
    @staticmethod
    def can_car_get_into_parking_slot(car, garage):
        left_cir_x, left_cir_y, _ = car.get_turnning_circle_center(left=True)
        right_cir_x, right_cir_y, _ = car.get_turnning_circle_center()
        step_0_edge = ReverseRelatedMethod.cal_edges_for_step_0(car, garage)
        left_step_1_edge = ReverseRelatedMethod.cal_edges_for_step_1(car, garage, left_cir_x, left_cir_y)
        right_step_1_edge = ReverseRelatedMethod.cal_edges_for_step_1(car, garage, right_cir_x, right_cir_y)
        if left_step_1_edge[0] is None and right_step_1_edge[0] is None and step_0_edge[0] is None:
            return False
        else:
            return True
    """

    @staticmethod
    def cal_turn_pos(car, start_pos, start_dir, end_pos, end_dir):
        n_start_dir = np.array([-start_dir[1], start_dir[0]])
        n_end_dir = np.array([-end_dir[1], end_dir[0]])

        if start_dir[0] > 0:
            n_start_dir = -n_start_dir
            n_end_dir = -n_end_dir

        para_start_pos = ReverseRelatedMethod.get_parallel_pos(start_pos, n_start_dir, car.get_efficient_min_r())
        para_end_pos = ReverseRelatedMethod.get_parallel_pos(end_pos, n_end_dir, car.get_efficient_min_r())

        para_pos_perpen_foot = np.dot(para_end_pos - para_start_pos, start_dir) * start_dir + para_start_pos

        para_end_to_foot = para_pos_perpen_foot - para_end_pos

        para_end_to_start = para_start_pos - para_end_pos

        circle_center = para_end_pos + (np.linalg.norm(para_end_to_foot)**2 / np.dot(end_dir, para_end_to_foot)) * end_dir # circle's center
        turn_pos = circle_center - car.get_efficient_min_r() * n_start_dir

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

        if np.abs(start_angle - end_angle) > np.pi:
            if start_angle < 0.0:
                start_angle += 2 * np.pi
            if end_angle < 0.0:
                end_angle += 2 * np.pi

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
    def __init__(self, theta, x, y, car_type='weight_center', length=4.6, width=1.8, wheelbase=2.6, min_r=5, max_steering_angle=30):

        # theta: the posture of the body of the car
        # x: the x coordinate of the car
        # y: the y coordinate of the car
        # car_type: 'weight_center', the turnning center of the car is determined by the weight center which is the default option
        # car_type: 'rear_axle_center', the turnning center of the car is determined by the rear axle center
        # length: the length of the car
        # width: the width of the car
        # min_r: the minimum turnning radius of the car, only work as the 'weight_center' type
        # wheelbase: the distance between two axles of the car
        # max_steering_angle: the maximum yaw angle of the car

        if car_type != 'weight_center' and car_type != 'rear_axle_center':
            print("wrong value of car_type ('weight_center' and 'rear_axle_center' only)")
            return None

        if min_r is None and max_steering_angle is None:
            print("min_r or max_steering_angle should not be None")
            return None

        self.theta = np.deg2rad(theta)
        self.x = x
        self.y = y
        self.cir_x = 0.0
        self.cir_y = 0.0
        self.phi = 0.0

        self.steering_wheel_angle = 0.0 # -1.0 to 1.0

        self.length = length
        self.width = width

        self.steer_angle = 0.0 # 0.0 to 1.0
        self.cur_rear_radius = -1.0
        self.cur_front_radius = -1.0
        self.min_r = min_r # only for the weight_center model, a coarse model

        self.continous_flag = False
        self.last_frame_status = 0 # -1, turn left, 0, straight, 1, turn right

        # for rear_axle center model, a fine model
        self.car_type = car_type
        self.wheelbase = wheelbase
        self.dis_wheels = width

        self.max_steering_angle = 0.0
        if max_steering_angle is None:
            self.max_steering_angle = np.arctan2(self.wheelbase, self.min_r)
        else:
            self.max_steering_angle = np.deg2rad(max_steering_angle)

        self.rear_min_r = self.wheelbase / np.tan(self.max_steering_angle)
        self.front_min_r = self.wheelbase / np.sin(self.max_steering_angle)

        self.efficient_min_r = self.rear_min_r
        # self.efficient_min_r = self.min_r if self.car_type == 'weight_center' else self.rear_min_r
        self.max_c = 1. / self.efficient_min_r

        self._observers = []
        self._state = None

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

    def add_observer(self, observer):
        self._observers.append(observer)

    def remove_observer(self, observer):
        self._observers.remove(observer)

    def set_state(self, state):
        self._state = state
        self._notify_observers()

    def get_state(self):
        return self._state

    def _notify_observers(self):
        for observer in self._observers:
            observer.update(self._state)

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

        wheel_0_x = mid_edge_0_x + dir_x * self.wheelbase * 0.5
        wheel_0_y = mid_edge_0_y + dir_y * self.wheelbase * 0.5
        wheel_1_x = mid_edge_0_x - dir_x * self.wheelbase * 0.5
        wheel_1_y = mid_edge_0_y - dir_y * self.wheelbase * 0.5
        wheel_2_x = mid_edge_1_x - dir_x * self.wheelbase * 0.5
        wheel_2_y = mid_edge_1_y - dir_y * self.wheelbase * 0.5
        wheel_3_x = mid_edge_1_x + dir_x * self.wheelbase * 0.5
        wheel_3_y = mid_edge_1_y + dir_y * self.wheelbase * 0.5

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
        arr_ed_x = self.x + dir_x * self.wheelbase
        arr_ed_y = self.y + dir_y * self.wheelbase

        mid_edge_0_x = self.x + self.wheelbase * dir_x / 2 + self.width * n_x / 2
        mid_edge_0_y = self.y + self.wheelbase * dir_y / 2 + self.width * n_y / 2
        mid_edge_1_x = self.x + self.wheelbase * dir_x / 2 - self.width * n_x / 2
        mid_edge_1_y = self.y + self.wheelbase * dir_y / 2 - self.width * n_y / 2

        vertex_0_x = mid_edge_0_x + dir_x * self.length * 0.5
        vertex_0_y = mid_edge_0_y + dir_y * self.length * 0.5
        vertex_1_x = mid_edge_0_x - dir_x * self.length * 0.5
        vertex_1_y = mid_edge_0_y - dir_y * self.length * 0.5
        vertex_2_x = mid_edge_1_x - dir_x * self.length * 0.5
        vertex_2_y = mid_edge_1_y - dir_y * self.length * 0.5
        vertex_3_x = mid_edge_1_x + dir_x * self.length * 0.5
        vertex_3_y = mid_edge_1_y + dir_y * self.length * 0.5

        wheel_0_x = mid_edge_0_x + dir_x * self.wheelbase * 0.5
        wheel_0_y = mid_edge_0_y + dir_y * self.wheelbase * 0.5
        wheel_1_x = mid_edge_0_x - dir_x * self.wheelbase * 0.5
        wheel_1_y = mid_edge_0_y - dir_y * self.wheelbase * 0.5
        wheel_2_x = mid_edge_1_x - dir_x * self.wheelbase * 0.5
        wheel_2_y = mid_edge_1_y - dir_y * self.wheelbase * 0.5
        wheel_3_x = mid_edge_1_x + dir_x * self.wheelbase * 0.5
        wheel_3_y = mid_edge_1_y + dir_y * self.wheelbase * 0.5

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

    def get_current_center(self, left=True):
        if left is True:
            phi = self.theta - np.pi / 2
            n_x = np.cos(self.theta + np.pi / 2)
            n_y = np.sin(self.theta + np.pi / 2)
            cir_x = self.x + n_x * self.efficient_min_r
            cir_y = self.y + n_y * self.efficient_min_r
        else:
            phi = self.theta + np.pi / 2
            n_x = np.cos(self.theta - np.pi / 2)
            n_y = np.sin(self.theta - np.pi / 2)
            cir_x = self.x + n_x * self.efficient_min_r
            cir_y = self.y + n_y * self.efficient_min_r

        return cir_x, cir_y, phi

    def get_turnning_circle_center(self, left=True):
        self.phi = 0.0
        if self.turn_left > self.turn_right or (left is True):
            self.phi = self.theta - np.pi / 2
            self.phi += self.reversing_status * self.angular_velocity
        else:
            self.phi = self.theta + np.pi / 2
            self.phi -= self.reversing_status * self.angular_velocity

        '''
        if self.continous_flag:
            return self.cir_x, self.cir_y, self.phi
        self.cir_x = 0.0
        self.cir_y = 0.0
        self.phi = 0.0
        '''
        if self.turn_left > self.turn_right or (left is True):
            n_x = np.cos(self.theta + np.pi / 2)
            n_y = np.sin(self.theta + np.pi / 2)
            self.cir_x = self.x + n_x * self.efficient_min_r
            self.cir_y = self.y + n_y * self.efficient_min_r
            '''
            self.phi = self.theta + np.pi / 2
            self.phi -= self.reversing_status * self.angular_velocity
            '''
        else:
            n_x = np.cos(self.theta - np.pi / 2)
            n_y = np.sin(self.theta - np.pi / 2)
            self.cir_x = self.x + n_x * self.efficient_min_r
            self.cir_y = self.y + n_y * self.efficient_min_r
            '''
            self.phi = self.theta - np.pi / 2
            self.phi += self.reversing_status * self.angular_velocity
            '''

        return self.cir_x, self.cir_y, self.phi

    def update_continous_flag(self):
        if self.turn_left == self.turn_right:
            if self.last_frame_status == 0:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = 0
        elif self.turn_left > self.turn_right:
            if self.last_frame_status == -1:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = -1
        else:
            if self.last_frame_status == 1:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = 1
        # print(self.last_frame_status, self.continous_flag)

    def update(self, frame):
        if self.turn_left == self.turn_right:
            '''
            if self.last_frame_status == 0:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = 0
            '''
            self.x += self.reversing_status * np.cos(self.theta) * self.velocity
            self.y += self.reversing_status * np.sin(self.theta) * self.velocity
            self.cir_x = 0.0
            self.cir_y = 0.0
        elif self.turn_left > self.turn_right:
            '''
            if self.last_frame_status == -1:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = -1
            '''
            cir_x, cir_y, phi = self.get_turnning_circle_center()
            # print('cir_x, cir_y, phi: ', cir_x, cir_y, phi, self.last_frame_status, self.continous_flag, self.last_frame_status)
            self.x = cir_x + self.efficient_min_r * np.cos(phi)
            self.y = cir_y + self.efficient_min_r * np.sin(phi)
            self.theta += self.reversing_status * self.angular_velocity
        else:
            '''
            if self.last_frame_status == 1:
                self.continous_flag = True
            else:
                self.continous_flag = False
            self.last_frame_status = 1
            '''
            cir_x, cir_y, phi = self.get_turnning_circle_center()
            # print('cir_x, cir_y, phi: ', cir_x, cir_y, phi, self.last_frame_status, self.continous_flag)
            self.x = cir_x + self.efficient_min_r * np.cos(phi)
            self.y = cir_y + self.efficient_min_r * np.sin(phi)
            self.theta -= self.reversing_status * self.angular_velocity

        self.set_state([self.x, self.y, self.theta, self.velocity, self.reversing_status, self.steering_wheel_angle])
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

    def update_status(self, x, y, yaw, v, reversing_status):
        self.x = x
        self.y = y
        self.theta = yaw
        self.velocity = v
        self.reversing_status = reversing_status

        if self.turn_left > self.turn_right:
            self.steering_wheel_angle = -1.0
        elif self.turn_left < self.turn_right:
            self.steering_wheel_angle = 1.0
        else:
            self.steering_wheel_angle = 0.0

    def get_width(self):
        return self.width

    def get_length(self):
        return self.length

    def get_dis_axles(self):
        return self.wheelbase

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

    def get_velocity(self):
        return self.velocity

    def get_steer_angle(self):
        return self.steer_angle

    def get_step_status(self):
        return self.step_status

    def get_efficient_min_r(self):
        return self.efficient_min_r

    def get_reversing_status(self):
        return self.reversing_status

    def get_left_status(self):
        return self.turn_left

    def get_right_status(self):
        return self.turn_right

    def set_theta(self, theta):
        self.theta = theta

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_steer_angle(self, steering_angle):
        self.steer_angle = steer_angle
        if np.abs(self.steer_angle - 0.0) < 0.01:
            self.steer_angle = 0.0
            self.cur_rear_radius = -1.0
            self.cur_front_radius = -1.0
        else:
            self.cur_rear_radius = self.wheelbase / np.tan(self.steer_angle)
            self.cur_front_radius = self.wheelbase / np.sin(self.steer_angle)

    def set_step_status(self, step_status):
        self.step_status = step_status

    def reset_status(self):
        self.step_status = 0

    def set_velocity(self, velocity):
        self.velocity = velocity
        self.angular_velocity = velocity / self.efficient_min_r

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

    def get_init_edge(self):
        return [np.array([0, self.l + 1.5]),
                np.array([0, self.l + self.s - 1.5]),
                np.array([self.h * 2 + self.w, self.l + 1.5]),
                np.array([self.h * 2 + self.w, self.l + self.s - 1.5])]

    def get_vertex(self):
        return [np.array([self.h + self.w, self.l]),
                np.array([self.h + self.w, 0]),
                np.array([self.h, 0]),
                np.array([self.h, self.l])]

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

