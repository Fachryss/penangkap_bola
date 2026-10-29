"""camera_node: baca webcam / file video, publish ke /camera/image_raw."""
import cv2
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # dynamic_typing supaya "-p source:=0" (int) maupun "0" (string) sama-sama bisa
        self.declare_parameter('source', '0',
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('mirror', False)

        self.source = str(self.get_parameter('source').value)
        self.fps = float(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.mirror = bool(self.get_parameter('mirror').value)

        self.is_video_file = not self.source.isdigit()
        if self.is_video_file:
            self.cap = cv2.VideoCapture(self.source)
        else:
            self.cap = cv2.VideoCapture(int(self.source))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Gagal membuka sumber: {self.source}')
            raise RuntimeError('Sumber gambar tidak bisa dibuka')

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / self.fps, self.tick)
        self.get_logger().info(
            f'camera_node jalan: source={self.source} fps={self.fps} '
            f'{self.width}x{self.height} video_file={self.is_video_file}')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.is_video_file:  # loop video dari awal
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Gagal membaca frame')
                return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        if self.mirror:
            frame = cv2.flip(frame, 1)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
