from typing import List, Tuple, NamedTuple
import time

from evasdk import Eva

class XYPoint(NamedTuple):
    x: int
    y: int

Grid_corners = Tuple[XYPoint, XYPoint, XYPoint]

class Grid2D:
    """
    Grid2D is a iterable collection of XYPoints generated from a set of grid 
    corner positions and a number of rows and columns
    """
    def __init__(self, grid_corners: Grid_corners, rows: int, columns: int):
        self.__current_position: int = 0
        self.__positions: List[XYPoint] = []

        x_start = grid_corners[0][0]
        x_end = grid_corners[1][0]

        y_start = grid_corners[1][1]
        y_end = grid_corners[2][1]
        
        column_step: float = (x_start - x_end) / (columns  - 1)
        row_step: float = (y_start - y_end) / (rows - 1)

        for column in range(columns):
            for row in range(rows):
                x = x_start - (column_step * column)
                y = y_start - (row_step * row)
                self.__positions.append(XYPoint(x = x, y = y))


    def __iter__(self):
        return self


    def __next__(self) -> XYPoint:
        if self.__current_position > len(self.__positions) - 1:
            raise StopIteration
        else:
            next_position = self.__positions[self.__current_position]
            self.__current_position += 1
            return next_position


grid_corners: Grid_corners = [
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



