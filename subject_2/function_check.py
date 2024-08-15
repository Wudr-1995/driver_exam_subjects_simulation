import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from path_planning_for_reversing import PathPlanningMethod
from simulation_element import ReverseRelatedMethod, Car, ReverseGarage, ReverseTrajectoryHandler

car_l = 4.6
car_w = 1.8
min_r = 5
dis_axle = 2.6

start_status = [8.1, 5.6, 90.0]
# start_status = [1.9943301547061152, 8.384488063516272, np.rad2deg(3.1622005175104473)]
# end_status = [8.1, 1.0, 90]

m_car = Car(start_status[2], start_status[0], start_status[1], 'rear_axle_center', car_l, car_w, dis_axle, min_r)
m_garage = ReverseGarage(car_l, car_w)
# m_path_planner = PathPlanningMethod(m_car, start_status, end_status)

"""
points = m_path_planner.plan_path()
vertexs, arrow, wheels = m_car.get_vertex()
edges, radii = m_path_planner.reversing_path_range(vertexs, arrow, wheels)
trajectory_edge_handler = []
"""

fig, ax = plt.subplots()
ax.grid(True)
m_garage.plot(ax)
m_car.plot(ax)

"""
trajectory_handler = ReverseTrajectoryHandler(ax, points, points[4], m_car.get_efficient_min_r())
for i in range(len(edges)):
    trajectory_edge_handler.append(ReverseTrajectoryHandler(ax, edges[i], points[4], radii[i], 0.3))
"""

step_0_left_edge, step_0_right_edge = ReverseRelatedMethod.cal_edges_for_step_0(m_car, m_garage, left=True)
if step_0_left_edge is not None:
    print('The edges of step 0 are legal ', step_0_left_edge, step_0_right_edge)
    # ax.scatter([step_0_left_edge[0]], [step_0_left_edge[1]], color='blue')
    # ax.scatter([step_0_right_edge[0]], [step_0_right_edge[1]], color='red')
else:
    print('step 0 edges are illegal')

cir_x, cir_y, phi = m_car.get_turnning_circle_center(left=True)
step_1_upper_edge, step_1_lower_edge = ReverseRelatedMethod.cal_edges_for_step_1(m_car, m_garage, cir_x, cir_y)
if step_1_lower_edge is not None:
    print('The edges of step 1 are legal ', step_1_upper_edge, step_1_lower_edge)
    ax.scatter([step_1_upper_edge[0]], [step_1_upper_edge[1]], color='blue')
    ax.scatter([step_1_lower_edge[0]], [step_1_lower_edge[1]], color='red')
else:
    print('step 1 edges are illegal')

step_3_upper_edge, step_3_lower_edge = ReverseRelatedMethod.cal_edges_for_step_3(m_car, m_garage, left=True)
if step_3_lower_edge is not None:
    print('The edges of step 3 are legal ', step_3_upper_edge, step_3_lower_edge)
    ax.scatter([step_3_upper_edge[0]], [step_3_upper_edge[1]], color='blue')
    ax.scatter([step_3_lower_edge[0]], [step_3_lower_edge[1]], color='red')
else:
    print('step 3 edges are illegal')

print('In the garage? ', ReverseRelatedMethod.if_in_the_garage(m_car, m_garage))
print('In the initial range? ', ReverseRelatedMethod.if_in_the_initial_range(m_car, m_garage))
print('Intersections? ', ReverseRelatedMethod.check_intersections(m_car, m_garage))
print('Can the car reach the initial range straight? ', ReverseRelatedMethod.car_can_reach_the_initial_straight(m_car, m_garage, left=True))
print('Can the car get into the garage? ', ReverseRelatedMethod.car_can_get_into_the_garage(m_car, m_garage))

plt.show()