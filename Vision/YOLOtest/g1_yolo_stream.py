import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

# 1. Ladda modellen lokalt på Jetson
print("Laddar YOLO-modellen...")
model = YOLO("best.pt")

# Global bildbuffer för streaming
output_frame = None
lock = threading.Lock()

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                with lock:
                    if output_frame is None:
                        continue
                    # Koda till JPEG för snabb överföring över Ethernet
                    ret, encoded_img = cv2.imencode('.jpg', output_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ret:
                        continue
                    bytes_data = encoded_img.tobytes()

                self.wfile.write(b'--jpgboundary\r\n')
                self.send_header('Content-type', 'image/jpeg')
                self.send_header('Content-length', str(len(bytes_data)))
                self.end_headers()
                self.wfile.write(bytes_data)
                self.wfile.write(b'\r\n')
                time.sleep(0.01)
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body><h1>Unitree G1 YOLO Stream</h1><img src='/stream.mjpg'/></body></html>")

def start_server():
    server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
    print("Stream tillgänglig på http://192.168.123.164:8080")
    server.serve_forever()

# Starta webbservern i en bakgrundstråd
server_thread = threading.Thread(target=start_server, daemon=True)
server_thread.start()

# 2. Initiera RealSense D435i kameran
print("Startar RealSense pipeline...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    print("Kör inferens live...")
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Konvertera till NumPy array
        frame = np.asanyarray(color_frame.get_data())

        # Kör YOLO-segmentering
        results = model.predict(frame, conf=0.5, verbose=False)

        # Rita ut masker och boxar på bilden
        annotated_frame = results[0].plot()

        with lock:
            output_frame = annotated_frame.copy()

except KeyboardInterrupt:
    print("Avslutar...")
finally:
    pipeline.stop()