"""game_node: deteksi objek berwarna (HSV) + game Bouncing Ball Catcher."""
import math
import random

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

BALL_COLORS = [(0, 0, 255), (255, 0, 0), (0, 165, 255), (255, 0, 255),
               (0, 255, 255), (255, 255, 0)]


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Konversi sensor_msgs/Image -> BGR numpy (tanpa cv_bridge)."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('bgr8', 'rgb8'):
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3).copy()
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if msg.encoding == 'mono8':
        img = data.reshape(msg.height, msg.step)[:, :msg.width].copy()
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'Encoding tidak didukung: {msg.encoding}')


def bgr_to_imgmsg(img: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg


class Ball:
    def __init__(self, color):
        self.x = self.y = self.vx = self.vy = 0.0
        self.color = color
        self.timer = 0.0


class GameNode(Node):
    def __init__(self):
        super().__init__('game_node')
        self.declare_parameter('hsv_lower', [40, 80, 80])
        self.declare_parameter('hsv_upper', [80, 255, 255])
        self.declare_parameter('num_balls', 3)
        self.declare_parameter('hold_duration', 2.0)
        self.declare_parameter('ball_radius', 40)
        self.declare_parameter('ball_speed', 150.0)
        self.declare_parameter('min_area', 300)
        self.declare_parameter('smoothing_alpha', 0.5)
        self.declare_parameter('speed_increase_per_score', 0.0)

        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        self.num_balls = int(self.get_parameter('num_balls').value)
        self.hold_duration = float(self.get_parameter('hold_duration').value)
        self.radius = int(self.get_parameter('ball_radius').value)
        self.base_speed = float(self.get_parameter('ball_speed').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.speed_inc = float(self.get_parameter('speed_increase_per_score').value)

        self.score = 0
        self.balls = []
        self.frame_w = self.frame_h = None
        self.last_stamp = None
        self.smoothed = None
        self.kernel = np.ones((5, 5), np.uint8)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)
        self.pub_score = self.create_publisher(Int32, '/game/score', 10)
        self.pub_debug = self.create_publisher(Image, '/game/debug_image', 10)
        self.srv_reset = self.create_service(Trigger, '/game/reset', self.on_reset)
        self.add_on_set_parameters_callback(self.on_params)

        self.publish_score()
        self.get_logger().info('game_node siap. Menunggu /camera/image_raw ...')

    # ---------- parameter dinamis (ros2 param set) ----------
    @property
    def speed(self):
        return self.base_speed + self.speed_inc * self.score

    def on_params(self, params):
        for p in params:
            n, v = p.name, p.value
            try:
                if n in ('hsv_lower', 'hsv_upper'):
                    if len(v) != 3:
                        return SetParametersResult(successful=False,
                                                   reason=f'{n} harus 3 elemen')
                    arr = np.array(v, dtype=np.uint8)
                    if n == 'hsv_lower':
                        self.hsv_lower = arr
                    else:
                        self.hsv_upper = arr
                elif n == 'num_balls':
                    self.num_balls = max(0, int(v))
                    self.sync_ball_count()
                elif n == 'hold_duration':
                    self.hold_duration = max(0.1, float(v))
                elif n == 'ball_radius':
                    self.radius = max(5, int(v))
                elif n == 'ball_speed':
                    self.base_speed = max(0.0, float(v))
                    self.rescale_velocities()
                elif n == 'min_area':
                    self.min_area = int(v)
                elif n == 'smoothing_alpha':
                    self.alpha = min(1.0, max(0.01, float(v)))
                elif n == 'speed_increase_per_score':
                    self.speed_inc = float(v)
                    self.rescale_velocities()
            except Exception as e:  # noqa
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ---------- bola ----------
    def respawn(self, b: Ball):
        r = self.radius
        b.x = random.uniform(r, max(r + 1, self.frame_w - r))
        b.y = random.uniform(r, max(r + 1, self.frame_h - r))
        ang = random.uniform(0, 2 * math.pi)
        b.vx = math.cos(ang) * self.speed
        b.vy = math.sin(ang) * self.speed
        b.timer = 0.0

    def sync_ball_count(self):
        if self.frame_w is None:
            return
        while len(self.balls) < self.num_balls:
            b = Ball(BALL_COLORS[len(self.balls) % len(BALL_COLORS)])
            self.respawn(b)
            self.balls.append(b)
        del self.balls[self.num_balls:]

    def rescale_velocities(self):
        for b in self.balls:
            mag = math.hypot(b.vx, b.vy)
            if mag > 1e-6:
                b.vx, b.vy = b.vx / mag * self.speed, b.vy / mag * self.speed

    def move_balls(self, dt):
        r = self.radius
        for b in self.balls:
            b.x += b.vx * dt
            b.y += b.vy * dt
            if b.x - r < 0:
                b.x, b.vx = r, abs(b.vx)
            elif b.x + r > self.frame_w:
                b.x, b.vx = self.frame_w - r, -abs(b.vx)
            if b.y - r < 0:
                b.y, b.vy = r, abs(b.vy)
            elif b.y + r > self.frame_h:
                b.y, b.vy = self.frame_h - r, -abs(b.vy)

    # ---------- deteksi ----------
    def detect(self, frame):
        blur = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.min_area:
            return None
        m = cv2.moments(c)
        if m['m00'] == 0:
            return None
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        return (cx, cy), cv2.boundingRect(c)

    # ---------- callback utama ----------
    def on_image(self, msg: Image):
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]
        if self.frame_w != w or self.frame_h != h:
            self.frame_w, self.frame_h = w, h
            self.balls = []
            self.sync_ball_count()

        # dt dari timestamp pesan (bukan hitung frame)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            dt = 0.0
        else:
            dt = stamp - self.last_stamp
            if dt < 0 or dt > 0.5:  # stamp aneh / frame putus lama
                dt = 0.0
        self.last_stamp = stamp

        # deteksi + smoothing
        det = self.detect(frame)
        centroid = None
        if det is not None:
            (cx, cy), (bx, by, bw, bh) = det
            if self.smoothed is None:
                self.smoothed = (cx, cy)
            else:
                a = self.alpha
                self.smoothed = (a * cx + (1 - a) * self.smoothed[0],
                                 a * cy + (1 - a) * self.smoothed[1])
            centroid = self.smoothed
        else:
            self.smoothed = None

        self.move_balls(dt)

        # tentukan bola yang terkena: hanya yang centroid-nya terdekat
        target = None
        if centroid is not None:
            best = None
            for b in self.balls:
                d = math.hypot(centroid[0] - b.x, centroid[1] - b.y)
                if d <= self.radius and (best is None or d < best):
                    best, target = d, b

        scored = False
        for b in self.balls:
            if b is target:
                b.timer += dt
                if b.timer >= self.hold_duration:
                    self.score += 1
                    scored = True
                    self.respawn(b)  # timer ikut di-reset
            else:
                b.timer = 0.0
        if scored:
            self.rescale_velocities()
            self.publish_score()

        # ---------- gambar ----------
        out = frame.copy()
        overlay = out.copy()
        for b in self.balls:
            cv2.circle(overlay, (int(b.x), int(b.y)), self.radius, b.color, -1)
        cv2.addWeighted(overlay, 0.45, out, 0.55, 0, out)
        for b in self.balls:
            p = (int(b.x), int(b.y))
            cv2.circle(out, p, self.radius, b.color, 2)
            if b.timer > 0:
                frac = min(1.0, b.timer / self.hold_duration)
                cv2.ellipse(out, p, (self.radius + 6, self.radius + 6), -90,
                            0, 360 * frac, (255, 255, 255), 4)
                cv2.putText(out, f'{b.timer:.1f}s', (p[0] - 18, p[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if det is not None:
            bx, by, bw, bh = det[1]
            cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            cv2.circle(out, (int(centroid[0]), int(centroid[1])), 5,
                       (0, 255, 0), -1)
            cv2.putText(out, 'OBJEK', (bx, max(15, by - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(out, f'Skor: {self.score}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4)
        cv2.putText(out, f'Skor: {self.score}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        self.pub_debug.publish(bgr_to_imgmsg(out, msg.header))

    def publish_score(self):
        m = Int32()
        m.data = self.score
        self.pub_score.publish(m)

    def on_reset(self, request, response):
        self.score = 0
        for b in self.balls:
            if self.frame_w:
                self.respawn(b)
        self.publish_score()
        response.success = True
        response.message = 'Game dan skor di-reset'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
