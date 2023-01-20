import numpy as np

class GridEnvironment:
    def __init__(self, grid):
        self.grid = grid

    def is_obstacle(self, pos):
        h, w = np.shape(self.grid)
        x = pos[0]
        y = pos[1]

        xl = x - 1 if x > 0 else 0
        xu = x + 2 if x < w else w
        yl = y - 1 if y > 0 else 0
        yu = y + 2 if y < h else h
        
        return not np.min(self.grid[yl:yu, xl:xu])
        
    def is_valid(self, pos):
        h, w = np.shape(self.grid)
        return pos[0] < w and pos[1] < h and pos[0] >= 0 and pos[1] >= 0
