from typing import List, Tuple, NamedTuple
import time

from evasdk import Eva
from grid2d import Grid2D, GridCorners, XYPoint

# Define the x and y coordinates for 3 corners of the grid
grid_corners: GridCorners = [
    XYPoint(x = 0.25, y = 0),
    XYPoint(x = 0.35, y = 0),
    XYPoint(x = 0.35, y = 0.4),
]
# Using the corners and an amount of rows and columns, make the Grid2D
my_test_grid = Grid2D(grid_corners, rows = 4, columns = 4)

# Connect to Eva
host_ip = input("Please enter a Eva IP: ")
token = input("Please enter a valid Eva token: ")
eva = Eva(host_ip, token)

# Set some default poses and a default orientation
pose_home = [0.057526037, 0.7658633, -1.9867575, 0.026749607, -1.732109, -0.011505207]
end_effector_orientation = {'w': 0.0, 'x': 0.0, 'y': 1.0, 'z': 0.0}
grid_z_position: float = 0.4

print("Waiting for Robot lock")
with eva.lock():
    print('Eva moving to home position')
    eva.control_go_to(pose_home)

    # For each grid position in the Grid2D
    for grid_position in my_test_grid:
        # Calculate joint angles for the grid position and goto those joint angles 
        print('Eva going to grid position x={:f}, y={:f}'.format(grid_position.x, grid_position.y))
        target_position = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position}
        position_joint_angles = eva.calc_inverse_kinematics(pose_home, target_position, end_effector_orientation)
        eva.control_go_to(position_joint_angles['ik']['joints'])

        # Simulating an action with a sleep, i.e. this could be picking from a pallet
        print('Eva performing action at grid waypoint')
        time.sleep(1)

        # Move back to the home position between each grid waypoint
        print('Eva moving to home position')
        eva.control_go_to(pose_home)

print("Grid movement complete, lock released")
