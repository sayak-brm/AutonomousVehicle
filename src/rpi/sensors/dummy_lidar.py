import numpy as np


class DummyLidar:
    def __init__(self):
        pass

    def start_scanning(self):
        pass

    def get_data(self):
        out = []
        for i in range(360):
            r = i
            t = np.radians(i)
            out.append((r, t))
        return out

    def stop_scanning(self):
        pass

    def close(self):
        pass
