import cv2
from flask import Flask, Response

app = Flask(__name__)
camera = cv2.VideoCapture(0)

# Снижаем разрешение для экономии трафика и ускорения передачи
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
camera.set(cv2.CAP_PROP_FPS, 15)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Конвертируем кадр в JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    print("[INFO] Запуск видеостримера...")
    print("[INFO] Телефон сможет получить видео по адресу: http://<IP_МАЛИНЫ>:5000/video_feed")
    # threaded=True позволяет нескольким клиентам подключаться одновременно
    app.run(host='0.0.0.0', port=5000, threaded=True)
