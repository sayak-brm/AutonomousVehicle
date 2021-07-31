#! /usr/bin/python3

from flask_restful import Resource
from flask_restful import Api
from flask import Flask, render_template, Response
import json

simulation = True

if not simulation:
    from rpi_camera import Camera
    from i2cio import I2CIO
else:
    from sim_camera import Camera

APP = Flask(__name__, template_folder='web', static_folder='static')
API = Api(APP)
if not simulation:
    I2C = I2CIO(0x01)
APP.config["DEBUG"] = True

opts = ""
with open("src/rpi/params.json") as params:
    opts = json.loads(params.read())


class Drive(Resource):
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
API.add_resource(Gears, "/api/gear/<number>")


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


if __name__ == "__main__":
    APP.run(port="8500", host="0.0.0.0")
