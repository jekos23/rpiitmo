import argparse
import atexit
import glob
import os
import shutil
import subprocess
import threading
import time

from flask import Flask, Response


app = Flask(__name__)

ffmpeg_process = None
ffmpeg_reader_thread = None
ffmpeg_stderr_thread = None
frame_condition = threading.Condition()
latest_frame = None
latest_frame_id = 0
stream_runtime_error = ""
camera_source_active = ""


def parse_camera_source(value):
    return str(value).strip()


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


def _set_stream_error(message):
    global stream_runtime_error
    stream_runtime_error = str(message or "").strip()


def _read_ffmpeg_stderr(process):
    if not process or not process.stderr:
        return

    try:
        for raw_line in iter(process.stderr.readline, b""):
            line = raw_line.decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[VIDEO] {line}")
    except Exception:
        pass


def _read_ffmpeg_stdout(process):
    global latest_frame, latest_frame_id

    if not process or not process.stdout:
        return

    buffer = bytearray()

    try:
        while process.poll() is None:
            chunk = process.stdout.read(32768)
            if not chunk:
                time.sleep(0.01)
                continue

            buffer.extend(chunk)

            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    if len(buffer) > 1048576:
                        del buffer[:-2]
                    break

                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start > 0:
                        del buffer[:start]
                    break

                frame = bytes(buffer[start : end + 2])
                del buffer[: end + 2]

                with frame_condition:
                    latest_frame = frame
                    latest_frame_id += 1
                    frame_condition.notify_all()
    except Exception as exc:
        _set_stream_error(f"FFmpeg frame reader failed: {exc}")


def _spawn_ffmpeg(command):
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    time.sleep(1.0)
    if process.poll() is not None:
        stderr_output = b""
        try:
            stderr_output = process.stderr.read().strip()
        except Exception:
            pass
        stderr_text = stderr_output.decode("utf-8", errors="ignore") if stderr_output else ""
        raise RuntimeError(stderr_text or "FFmpeg exited immediately.")
    return process


def _ffmpeg_command(camera_source, mjpeg_copy):
    ffmpeg_path = shutil.which("ffmpeg")
    if not ffmpeg_path:
        raise RuntimeError("FFmpeg is not installed.")

    command = [
        ffmpeg_path,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-f",
        "v4l2",
    ]

    if mjpeg_copy:
        command.extend(["-input_format", "mjpeg"])

    command.extend(
        [
            "-i",
            str(camera_source),
            "-an",
        ]
    )

    if mjpeg_copy:
        command.extend(["-c:v", "copy"])
    else:
        command.extend(["-c:v", "mjpeg", "-q:v", "4"])

    command.extend(["-f", "image2pipe", "pipe:1"])
    return command


def release_stream():
    global ffmpeg_process, ffmpeg_reader_thread, ffmpeg_stderr_thread

    process = ffmpeg_process
    ffmpeg_process = None

    if process is not None:
        try:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        except Exception:
            try:
                process.kill()
            except Exception:
                pass

    ffmpeg_reader_thread = None
    ffmpeg_stderr_thread = None


def init_stream(camera_source):
    global ffmpeg_process, ffmpeg_reader_thread, ffmpeg_stderr_thread
    global camera_source_active, latest_frame, latest_frame_id

    release_stream()

    requested_source = parse_camera_source(camera_source)
    attempts = [requested_source] if requested_source else []
    for candidate in find_camera_candidates():
        if candidate not in attempts:
            attempts.append(candidate)

    last_error = "No camera devices found."
    for attempt in attempts:
        for mjpeg_copy in (True, False):
            command = _ffmpeg_command(attempt, mjpeg_copy=mjpeg_copy)
            mode_label = "MJPEG copy" if mjpeg_copy else "MJPEG encode"
            try:
                print(f"[VIDEO] Trying {attempt} with {mode_label}.")
                process = _spawn_ffmpeg(command)
                ffmpeg_process = process
                camera_source_active = attempt
                latest_frame = None
                latest_frame_id = 0
                _set_stream_error("")

                ffmpeg_reader_thread = threading.Thread(
                    target=_read_ffmpeg_stdout,
                    args=(process,),
                    daemon=True,
                )
                ffmpeg_reader_thread.start()

                ffmpeg_stderr_thread = threading.Thread(
                    target=_read_ffmpeg_stderr,
                    args=(process,),
                    daemon=True,
                )
                ffmpeg_stderr_thread.start()

                if attempt != requested_source and requested_source:
                    print(f"[VIDEO] Camera auto-selected: {requested_source} -> {attempt}")
                print(f"[VIDEO] Streaming source ready: {attempt} ({mode_label}).")
                return
            except Exception as exc:
                last_error = str(exc)
                release_stream()

    _set_stream_error(last_error)
    raise RuntimeError(f"Failed to open camera: {requested_source}. {last_error}")


atexit.register(release_stream)


def generate_frames():
    local_frame_id = -1
    idle_loops = 0

    while True:
        with frame_condition:
            frame_condition.wait_for(
                lambda: latest_frame_id != local_frame_id or ffmpeg_process is None,
                timeout=2.0,
            )
            frame = latest_frame
            frame_id = latest_frame_id
            process = ffmpeg_process

        if frame is not None and frame_id != local_frame_id:
            local_frame_id = frame_id
            idle_loops = 0
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            continue

        if process is None or process.poll() is not None:
            idle_loops += 1
            if idle_loops >= 3:
                break


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def main():
    parser = argparse.ArgumentParser(
        description="Low-overhead MJPEG streamer for Raspberry Pi camera source"
    )
    parser.add_argument("--camera", default="/dev/video0", help="Camera source, e.g. /dev/video0")
    parser.add_argument("--host", default="0.0.0.0", help="Host for Flask server")
    parser.add_argument("--port", type=int, default=6767, help="HTTP port for MJPEG stream")
    args = parser.parse_args()

    print(f"[VIDEO] Opening camera: {args.camera}")
    init_stream(args.camera)
    print("[VIDEO] Streamer started without OpenCV.")
    print(f"[VIDEO] Stream is available at: http://<IP_PI>:{args.port}/video_feed")
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
