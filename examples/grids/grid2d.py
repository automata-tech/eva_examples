from typing import List, Tuple, NamedTuple

class XYPoint(NamedTuple):
    x: int
    y: int

GridCorners = Tuple[XYPoint, XYPoint, XYPoint]

class Grid2D:
    """
    Grid2D is a iterable collection of XYPoints generated from a set of grid 
    corner positions and a number of rows and columns
    """
    def __init__(self, grid_corners: GridCorners, rows: int, columns: int):
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
