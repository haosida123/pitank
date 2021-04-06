from flask import Flask, render_template, Response, request
from camera_opencv import Camera
# from camera import Camera
import time  # Import the Time library
import datetime
from move import Move

app = Flask(__name__)

move = None
setfps = 30
# movetime = 0.2


@app.route('/')
def index():
    message = "ahhhhhhh"
    return render_template('index.html', message=message)

# @app.route("/test")
# def test():
#     return render_template('test.html')

@app.route("/fps")
def fps():
    global setfps
    setfps = int(request.args.get('setfps'))
    return "OK"

@app.route("/stream/")
def stream():
    return render_template('stream.html')


def gen(camera):
    global setfps
    while True:
        # print(str(datetime.datetime.now())[:21])
        frame = camera.get_frame(setfps)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/forward")
def move_forward():
    speed_pct = int(request.args.get('speed_pct'))
    move.speed = speed_pct
    move.move('forward', 'no')
    return "OK"

@app.route("/backward")
def move_backward():
    speed_pct = int(request.args.get('speed_pct'))
    move.speed = speed_pct
    move.move('backward', 'no')
    return "OK"

@app.route("/turn_right")
def turn_right():
    speed_pct = int(request.args.get('speed_pct'))
    move.speed = speed_pct
    move.move('no', 'right')
    return "OK"

@app.route("/turn_left")
def turn_left():
    speed_pct = int(request.args.get('speed_pct'))
    move.speed = speed_pct
    move.move('no', 'left')
    return "OK"

@app.route("/camera_up")
def camera_up():
    move.servo.move("camera", "up")
    return "OK"

@app.route("/camera_down")
def camera_down():
    move.servo.move("camera", "down")
    return "OK"

@app.route("/stop")
def move_left():
    move.stop()
    move.servo.stopWiggle()
    forward_message = "Stopped..."
    return render_template('index.html', forward_message=forward_message)


if __name__ == '__main__':
    # global move
    # global movetime
    movetime = 0.2
    with Move() as move:
        # move.move('forward', 'no')
        # time.sleep(movetime)
        # move.stop()
        app.run(debug=True, host='0.0.0.0', threaded=True)
