import numpy as np
import matplotlib.pyplot as plt
# from path_planning_for_reversing import PathPlanningMethod
# from simulation_element import ReverseRelatedMethod, Car, ReverseGarage, AnimationController
from animation_controller import ReversingCarAnimationController

if __name__ == '__main__':
    car_l = 4.6
    car_w = 1.8
    min_r = 5
    dis_axle = 2.6

    '''
    m_garage = ReverseGarage(car_l, car_w)

    start_status = [0, m_garage.get_l() + m_garage.get_s() / 2, 180]
    end_status = [m_garage.get_h() + m_garage.get_w() / 2, (car_l - dis_axle) / 2 + 0.5, 90]

    m_car = Car(start_status[2], start_status[0], start_status[1], 'rear_axle_center', car_l, car_w, min_r)
    # m_car = Car(start_status[2], start_status[0], start_status[1], 'weight_center', car_l, car_w, min_r)

    path_planner = PathPlanningMethod(m_car, start_status, end_status)
    points = path_planner.plan_path()
    edges, radii = path_planner.reversing_path_range()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    m_garage.plot(ax)
    ReverseRelatedMethod.plot_trajectory(ax, points[0], points[1], points[2], points[3], points[4], m_car.get_efficient_min_r())
    for i in range(len(edges)):
        ReverseRelatedMethod.plot_trajectory(ax, edges[i][0], edges[i][1], edges[i][2], edges[i][3], points[4], radii[i], 0.3)
    m_controller = AnimationController(m_car, fig, ax)
    plt.show()
    '''

    # start_status = [0, 8.75, 178]
    start_status = [1.9943301547061152, 8.384488063516272, np.rad2deg(3.1622005175104473)]
    end_status = [8.1, 1.0, 90]

    m_controller = ReversingCarAnimationController(car_l, car_w, min_r, start_status, end_status)
    m_controller.show()
