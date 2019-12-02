from typing import List, Tuple, NamedTuple
import time

from evasdk import Eva
from grid2d import Grid2D, GridCorners, XYPoint

grid_corners: GridCorners = [
    XYPoint(x = 0, y = 0),
    XYPoint(x = 3, y = 0),
    XYPoint(x = 3, y = 5),
]
my_test_grid = Grid2D(grid_corners, rows = 4, columns = 4)

host_ip = input("Please enter a Eva IP: ")
token = input("Please enter a valid Eva token: ")
eva = Eva(host_ip, token)

pose_guess = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
pose_home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_effector_orientation = {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
grid_z_position = 2

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to(pose_home)

    for grid_position in my_test_grid:
        print('Eva going to grid position x={:f}, y={:f}'.format(grid_position.x, grid_position.y))
        target_position = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position}
        position_joint_angles = eva.calc_inverse_kinematics(pose_guess, target_position, end_effector_orientation)
        eva.control_go_to(position_joint_angles['ik']['joints'])

        # simulating an action with a sleep, i.e. picking up something from a pallet
        print('Eva performing action at grid waypoint')
        time.sleep(1)

        print('Eva moving to home position')
        eva.control_go_to(pose_home)



