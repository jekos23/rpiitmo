import argparse
import atexit

import cv2
from flask import Flask, Response


app = Flask(__name__)
camera = None


def parse_camera_source(value):
    value = str(value).strip()
    if value.isdigit():
        return int(value)
    return value


def init_camera(camera_source):
    global camera

    source = parse_camera_source(camera_source)
    camera = cv2.VideoCapture(source)

    if not camera.isOpened():
        raise RuntimeError(f"Не удалось открыть камеру: {camera_source}")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    camera.set(cv2.CAP_PROP_FPS, 15)


def release_camera():
    global camera
    if camera is not None:
        try:
            camera.release()
        except Exception:
            pass
        camera = None


atexit.register(release_camera)


def generate_frames():
    while True:
        if camera is None:
            break

        success, frame = camera.read()
        if not success:
            continue

        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            continue

        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    parser = argparse.ArgumentParser(description="MJPEG streamer for Raspberry Pi camera source")
    parser.add_argument("--camera", default="0", help="Camera source, e.g. 0 or /dev/video0")
    parser.add_argument("--host", default="0.0.0.0", help="Host for Flask server")
    parser.add_argument("--port", type=int, default=5000, help="HTTP port for MJPEG stream")
    args = parser.parse_args()

    print(f"[VIDEO] Открываю камеру: {args.camera}")
    init_camera(args.camera)
    print("[VIDEO] Видеостример запущен.")
    print(f"[VIDEO] Поток доступен по адресу: http://<IP_PI>:{args.port}/video_feed")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
