#! /usr/bin/python3

from flask_restful import Resource
from flask_restful import Api
from flask import Flask, render_template, Response
import json
import matplotlib.image as img
import numpy as np
import pygame
from pygame import gfxdraw
import threading

from sparse_matrix import SparseMatrix

simulation = True

if not simulation:
    from rpi_camera import Camera
    from sensors.simulated_lidar import SimulatedLidar as Lidar
    from i2cio import I2CIO
else:
    from sim_camera import Camera
    from sensors.simulated_lidar import SimulatedLidar as Lidar

APP = Flask(__name__, template_folder='web', static_folder='static')
API = Api(APP)
if not simulation:
    I2C = I2CIO(0x01)
APP.config["DEBUG"] = True

spmat = SparseMatrix(0.5)
x, y, t = 40, 40, 0
running = True
map_scale = 3
disp_scale = 10
manual_dir = "stop"

opts = ""
with open("src/rpi/params.json") as params:
    opts = json.loads(params.read())


def map_data(data, Ox=0, Oy=0, Ot=0, max_dist=np.inf, scale=3):
    v = spmat.get(Ox//scale, Oy//scale)/2
    spmat.update(Ox//scale, Oy//scale, v)
    for r, t in data:
        for i in range(int(r)):
            if i < max_dist:
                x = i * np.cos(t + np.radians(Ot))
                y = i * np.sin(t + np.radians(Ot))
                v = spmat.get((Ox+x)//scale, (Oy+y)//scale)/2
                spmat.update((Ox+x)//scale, (Oy+y)//scale, v)
        if r < max_dist:
            x = r * np.cos(t + np.radians(Ot))
            y = r * np.sin(t + np.radians(Ot))
            v = spmat.get((Ox+x)//scale, (Oy+y)//scale)/2 + 0.5
            spmat.update((Ox+x)//scale, (Oy+y)//scale, v)


def draw_rect(screen, map_x, map_y, col, scale=10):
    for i in range(scale):
        for j in range(scale):
            gfxdraw.pixel(screen, map_x*scale+i, map_y*scale+j, col)


def run_lidar(lidar, screen, map_scale=3, disp_scale=10):
    global running, x, y, t
    while running:
        if manual_dir != "stop":
            if manual_dir == "left":
                x -= map_scale
            if manual_dir == "right":
                x += map_scale
            if manual_dir == "front":
                y -= map_scale
            if manual_dir == "back":
                y += map_scale

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    x -= map_scale
                if event.key == pygame.K_d:
                    x += map_scale
                if event.key == pygame.K_w:
                    y -= map_scale
                if event.key == pygame.K_s:
                    y += map_scale
                if event.key == pygame.K_q:
                    t -= map_scale
                if event.key == pygame.K_e:
                    t += map_scale
            if event.type == pygame.QUIT:
                running = False

        if not simulation:
            lidar_data = lidar.get_data()
        else:
            lidar_data = lidar.get_data(x, y, t)
        map_data(lidar_data, x, y, t, max_dist=100, scale=map_scale)

        w, h = screen.get_size()
        c_x = w//disp_scale//2
        c_y = h//disp_scale//2
        b_x = c_x - x//map_scale
        b_y = c_y - y//map_scale

        screen.fill((127, 127, 127))

        for map_x in spmat.head.keys():
            for map_y in spmat.head[map_x].keys():
                col = int(255*(1-spmat.head[map_x][map_y]))
                draw_rect(screen, int(map_x + b_x), int(map_y + b_y),
                          (col, col, col), scale=disp_scale)

        draw_rect(screen, c_x, c_y, (0, 255, 0), scale=disp_scale)

        pygame.display.flip()


""" class Drive(Resource):
    @staticmethod
    def post(direction):
        if direction in opts["dirs"]:
            if not simulation:
                I2C.send_write([6, opts["dirs"][direction]])
            return {"success": True}
        print("ERR: Incorrect Drive Direction")
        return {"success": False}


class Lights(Resource):
    @staticmethod
    def post(light, state):
        if light in opts["lights"]:
            if state in opts["relay_ao"]:
                if not simulation:
                    I2C.send_write(
                        [2, opts["lights"][light], opts["relay_ao"][state]])
                return {"success": True}
        return {"success": False}


class Gears(Resource):
    @staticmethod
    def post(number):
        if number in opts["gears"]:
            if not simulation:
                I2C.send_write([7, opts["gears"][number]])
            return {"success": True}
        return {"success": False}


API.add_resource(Drive, "/api/drive/<direction>")
API.add_resource(Lights, "/api/lights/<light>/<state>")
API.add_resource(Gears, "/api/gear/<number>") """


class Drive(Resource):
    @staticmethod
    def post(direction):
        global manual_dir
        if direction in opts["dirs"]:
            if not simulation:
                I2C.send_write([6, opts["dirs"][direction]])
            manual_dir = direction
            return {"success": True}
        print("ERR: Incorrect Drive Direction")
        return {"success": False}


API.add_resource(Drive, "/api/drive/<direction>")


@APP.route('/')
def index():
    return render_template('index.html')


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@APP.route('/camera')
def camera():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def init_lidar():
    if not simulation:
        lidar = Lidar()
    else:
        map_image = img.imread("src/rpi/sensors/maps/1.png")[:, :, 0]
        lidar = Lidar(map_image)

    lidar.start_scanning()

    pygame.init()
    screen = pygame.display.set_mode((720, 720), pygame.RESIZABLE)

    run_lidar(lidar, screen, map_scale=map_scale, disp_scale=disp_scale)

    lidar.stop_scanning()
    lidar.close()


if __name__ == "__main__":
    server = threading.Thread(target=init_lidar)
    server.start()
    APP.run(port="8500", host="0.0.0.0", use_reloader=False)
    running = False
    server.join()
