from time import time


class Camera(object):
    def __init__(self):
        self.frames = [open('src/rpi/sim_camera/images/' +
                            f + '.jpg', 'rb').read() for f in ['1', '2', '3']]

    def get_frame(self):
        return self.frames[int(time()) % 3]
