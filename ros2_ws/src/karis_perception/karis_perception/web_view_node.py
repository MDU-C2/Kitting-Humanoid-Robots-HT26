"""Shows ROS CompressedImage topics live in a web browser (MJPEG), e.g. the perception's
annotated image and the detector's masks on a computer without a screen (the Eskilstuna PC over ssh).

    ros2 run karis_perception web_view                     # http://<this computer>:8081/
    ros2 run karis_perception web_view --ros-args -p port:=8082 -p topics:="['/karis/viz/annotated/compressed']"
    ros2 launch karis_bringup perception.launch.py ... web_port:=8081

Pages: /  (all topics), /stream/<n> (MJPEG of topic n), /snapshot/<n>.jpg (one frame).
A topic is only subscribed while a browser shows it, so the perception only draws and encodes
its annotated image while someone watches. Anyone who can reach the port can watch.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
DEFAULT_TOPICS = ["/karis/viz/annotated/compressed", "/karis/viz/masks/compressed",
                  "/g1/head_camera/color/image_raw/compressed"]


class Feed:
    """Latest JPEG of one topic; subscribed while at least one browser watches."""

    def __init__(self, node, topic):
        self.node, self.topic = node, topic
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.t_last = None, 0, 0.0
        self.times = []                  # receive times for the frame rate
        self.viewers, self.sub = 0, None

    def watch(self, on):
        with self.cond:
            self.viewers += 1 if on else -1
            want = self.viewers > 0
        if want and self.sub is None:
            self.sub = self.node.create_subscription(CompressedImage, self.topic, self._on, IMAGE_QOS)
            self.node.get_logger().info(f"watching {self.topic}")
        elif not want and self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None
            self.node.get_logger().info(f"nobody watches {self.topic}: unsubscribed")

    def _on(self, msg):
        data = bytes(msg.data)
        if "jpeg" not in msg.format.lower() and "jpg" not in msg.format.lower():
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)   # png etc.
            if img is None:
                return
            data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        now = time.monotonic()
        with self.cond:
            self.jpeg, self.t_last = data, now
            self.seq += 1
            self.times = [t for t in self.times if now - t < 2.0] + [now]
            self.cond.notify_all()

    def status(self):
        with self.cond:
            if self.jpeg is None:
                return "no image yet" if self.sub else "not subscribed"
            age = time.monotonic() - self.t_last
            fps = len(self.times) / 2.0 if age < 2.0 else 0.0
            return f"{fps:.1f} frames/s, last {age:.1f} s ago"


def make_handler(feeds):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):           # quiet
            pass

        def _feed(self, part):
            try:
                return feeds[int(part)]
            except (ValueError, IndexError):
                self.send_error(404, "no such topic")
                return None

        def do_GET(self):
            path = urlparse(self.path).path.strip("/").split("/")
            if path == [""]:
                return self._index()
            if path[0] == "status":
                return self._text("\n".join(f"{i} {f.topic}: {f.status()}" for i, f in enumerate(feeds)))
            if len(path) == 2 and path[0] == "stream":
                f = self._feed(path[1])
                return f and self._stream(f)
            if len(path) == 2 and path[0] == "snapshot":
                f = self._feed(path[1].split(".")[0])
                return f and self._snapshot(f)
            self.send_error(404)

        def _text(self, txt, ctype="text/plain"):
            body = txt.encode()
            self.send_response(200)
            self.send_header("Content-Type", f"{ctype}; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _index(self):
            items = "".join(
                f'<section><h2>{f.topic}</h2><p class="st" data-i="{i}">…</p>'
                f'<img src="/stream/{i}" alt="{f.topic}"></section>' for i, f in enumerate(feeds))
            self._text(f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>KARIS live view</title>
<style>body{{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#ddd}}
h1{{font-size:1.2rem}} h2{{font-size:1rem;margin:16px 0 4px;font-family:monospace}}
img{{max-width:100%;border:1px solid #444;background:#000}} .st{{margin:0 0 6px;font-size:.85rem;color:#9c9}}</style>
</head><body><h1>KARIS live view</h1>{items}
<script>setInterval(async()=>{{try{{const t=(await (await fetch('/status')).text()).split('\\n');
document.querySelectorAll('.st').forEach(e=>{{const l=t[+e.dataset.i]||'';e.textContent=l.slice(l.indexOf(': ')+2)}})}}catch(e){{}}}},1000)</script>
</body></html>""", "text/html")

        def _snapshot(self, f):
            f.watch(True)
            try:
                with f.cond:
                    f.cond.wait_for(lambda: f.jpeg is not None, timeout=5.0)
                    jpeg = f.jpeg
            finally:
                f.watch(False)
            if jpeg is None:
                return self.send_error(503, f"no image on {f.topic}")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def _stream(self, f):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            f.watch(True)
            seq = -1
            try:
                while True:
                    with f.cond:
                        f.cond.wait_for(lambda: f.seq != seq, timeout=5.0)
                        if f.jpeg is None:
                            continue                     # nothing yet: keep waiting
                        # a new frame, or after 5 s the last one again (notices a closed browser)
                        seq, jpeg = f.seq, f.jpeg
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                                     # browser closed
            finally:
                f.watch(False)

    return Handler


class WebView(Node):
    def __init__(self):
        super().__init__("karis_web_view")
        self.port = int(self.declare_parameter("port", 8081).value)
        self.bind = str(self.declare_parameter("bind", "0.0.0.0").value)
        topics = list(self.declare_parameter("topics", DEFAULT_TOPICS).value)
        self.feeds = [Feed(self, t) for t in topics]


def main(args=None):
    rclpy.init(args=args)
    node = WebView()
    server = ThreadingHTTPServer((node.bind, node.port), make_handler(node.feeds))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f"web view on http://{node.bind}:{node.port}/ for "
                           f"{[f.topic for f in node.feeds]} (open http://<this computer's address>:{node.port}/)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
