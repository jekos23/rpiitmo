import argparse
import atexit
import glob
import os
import time

import cv2
from flask import Flask, Response


app = Flask(__name__)
camera = None


def parse_camera_source(value):
    value = str(value).strip()
    if value.isdigit():
        return int(value)
    return value


def _read_text_if_exists(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            return handle.read().strip()
    except OSError:
        return None


def find_camera_candidates():
    all_nodes = sorted(dict.fromkeys(glob.glob("/dev/video*")))
    if not all_nodes:
        return []

    preferred_nodes = []
    fallback_nodes = []

    for port in all_nodes:
        video_name = os.path.basename(port)
        index_text = _read_text_if_exists(
            os.path.join("/sys/class/video4linux", video_name, "index")
        )
        node_name = (
            _read_text_if_exists(
                os.path.join("/sys/class/video4linux", video_name, "name")
            )
            or ""
        ).lower()

        if any(
            token in node_name
            for token in ("metadata", "codec", "isp", "stats", "raw", "subdev")
        ):
            continue

        try:
            index_value = int(index_text) if index_text is not None else None
        except (TypeError, ValueError):
            index_value = None

        if index_value in (None, 0):
            preferred_nodes.append(port)
        else:
            fallback_nodes.append(port)

    return preferred_nodes or fallback_nodes or all_nodes


def _open_camera_handle(source):
    try:
        return cv2.VideoCapture(source, cv2.CAP_V4L2)
    except Exception:
        return cv2.VideoCapture(source)


def _try_camera_source(camera_source):
    source = parse_camera_source(camera_source)
    handle = _open_camera_handle(source)
    if not handle or not handle.isOpened():
        if handle:
            handle.release()
        return None

    handle.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    handle.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    handle.set(cv2.CAP_PROP_FPS, 15)

    for _ in range(6):
        ok, frame = handle.read()
        if ok and frame is not None:
            return handle
        time.sleep(0.12)

    handle.release()
    return None


def init_camera(camera_source):
    global camera

    requested_source = str(camera_source).strip()
    attempts = [requested_source] if requested_source else []

    for candidate in find_camera_candidates():
        if candidate not in attempts:
            attempts.append(candidate)

    for attempt in attempts:
        handle = _try_camera_source(attempt)
        if handle is not None:
            camera = handle
            if attempt != requested_source:
                print(f"[VIDEO] Camera auto-selected: {requested_source} -> {attempt}")
            return

    raise RuntimeError(f"Failed to open camera: {camera_source}")


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
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def main():
    parser = argparse.ArgumentParser(
        description="MJPEG streamer for Raspberry Pi camera source"
    )
    parser.add_argument("--camera", default="0", help="Camera source, e.g. 0 or /dev/video0")
    parser.add_argument("--host", default="0.0.0.0", help="Host for Flask server")
    parser.add_argument("--port", type=int, default=6767, help="HTTP port for MJPEG stream")
    args = parser.parse_args()

    print(f"[VIDEO] Opening camera: {args.camera}")
    init_camera(args.camera)
    print("[VIDEO] Streamer started.")
    print(f"[VIDEO] Stream is available at: http://<IP_PI>:{args.port}/video_feed")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
