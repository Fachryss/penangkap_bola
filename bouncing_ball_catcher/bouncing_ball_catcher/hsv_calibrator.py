"""Tool kalibrasi HSV dengan trackbar. Subscribe /camera/image_raw.
Geser trackbar sampai hanya objekmu yang putih di jendela Mask.
Tekan 'p' untuk cetak nilai; 'q' untuk keluar."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from bouncing_ball_catcher.game_node import imgmsg_to_bgr

WIN = 'HSV Calibrator'


class HsvCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.frame = None
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        cv2.namedWindow(WIN)
        for name, init, mx in [('H_low', 40, 179), ('S_low', 80, 255),
                               ('V_low', 80, 255), ('H_high', 80, 179),
                               ('S_high', 255, 255), ('V_high', 255, 255)]:
            cv2.createTrackbar(name, WIN, init, mx, lambda x: None)
        self.create_timer(0.03, self.loop)

    def cb(self, msg):
        self.frame = imgmsg_to_bgr(msg)

    def values(self):
        g = lambda n: cv2.getTrackbarPos(n, WIN)  # noqa
        return ([g('H_low'), g('S_low'), g('V_low')],
                [g('H_high'), g('S_high'), g('V_high')])

    def loop(self):
        if self.frame is None:
            cv2.waitKey(1)
            return
        lo, hi = self.values()
        hsv = cv2.cvtColor(cv2.GaussianBlur(self.frame, (7, 7), 0),
                           cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        result = cv2.bitwise_and(self.frame, self.frame, mask=mask)
        cv2.imshow(WIN, np.hstack([self.frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord('p'):
            self.get_logger().info(f'hsv_lower: {lo}  hsv_upper: {hi}')
        elif k == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HsvCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
