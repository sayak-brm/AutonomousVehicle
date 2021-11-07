import math


class SparseMatrix:
    """
    Sparse matrix data structure with support for negative indexes.
    Attributes:
        head (dict): Dictionary containing the sparse matrix.
        default (any): Default value of the sparse matrix.
    """

    def __init__(self, default=0, head={}):
        """
        Initialize the sparse matrix.
        Args:
            default (any, optional): Default value of the sparse matrix. Defaults to 0.
            head (dict, optional): Dictionary containing the sparse matrix. Defaults to {}.
        """
        self.head = head
        self.default = default

    def update(self, x, y, val):
        """
        Update a position in the sparse matrix.
        Args:
            x (int): Index of the top-level dict.
            y (int): Index of the nested dict.
            val (any): New value of the position to be updated.
        """
        if val == self.default:
            if x in self.head and y in self.head[x]:
                del self.head[x][y]
                if self.head[x] == {}:
                    del self.head[x]
        else:
            if x in self.head:
                self.head[x][y] = val
            else:
                self.head[x] = {y: val}

    def get(self, x, y):
        """
        Get a value from a particular position.
        Args:
            x (int): Index of top-level dict.
            y (int): Index of nested dict.
        Returns:
            [type]: [description]
        """
        if x in self.head and y in self.head[x]:
            return self.head[x][y]
        else:
            return self.default

    def get_max_dist(self, Ox, Oy):
        max_coords = (Ox, Oy)
        for x in self.head.keys():
            for y in self.head[x].keys():
                max_coords = max(max_coords, (x, y),
                                 key=lambda tup: math.hypot(tup[0]-Ox, tup[1]-Oy))
        return math.hypot(max_coords[0]-Ox, max_coords[1]-Oy)
