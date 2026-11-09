"""Find the stem above the nearest ripe fruit in the wrist RGB-D image.

The field camera sees where the fruit is, but not where its stem is, so
harvest_demo would otherwise assume the stem goes straight up. From the
pre-approach pose, about 15 cm in front of the stem, the wrist camera sees
the fruit top and the stem clearly. For each color image the node:

1. Finds the largest ripe (red) blob and its 3D center, as fruit_detector
   does (nearest depth plus the fruit radius along the centroid ray).
2. Keeps the stem colored pixels (pale green: hue like the unripe fruit but
   low saturation) and turns them into 3D points in the target frame. The
   visible surface is one stem radius in front of the stem axis, so each
   point is moved that far along its viewing ray.
3. Keeps the points near the fruit: within region_radius sideways of its
   center and from its top to stem_height above the top.
4. Fits a line to them (principal axis) and returns the point on it
   grasp_above over the fruit top, where the fingers should close.

A leaf can hide the stem, or leave only a short piece of it in view, and
a line through a few centimetres of points or through stray pixels is not
a stem. The node publishes nothing (and warns why) unless there are
min_stem_points, they span at least min_span along the line, and their
distance from it is at most max_rms. harvest_demo then looks again from
another side.

The result goes to ~/grasp as a PoseStamped: position = grasp point,
orientation = rotation from +z to the stem direction (pointing up). The
tilt from vertical, the fruit center and the number of stem points are
logged. An annotated image goes to ~/image.

The transform of the camera is the latest one, so the arm must be still
when the image is taken (harvest_demo waits for an image after it stops).
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from agri_perception.fruit_detector import CLASSES, MIN_SATURATION, MIN_VALUE, quat_to_matrix

# Stem: hue between yellow and green, low saturation (OpenCV hue 0-179).
STEM_HUE = (35, 75)
STEM_SATURATION = (40, MIN_SATURATION - 1)


def quat_from_z(d):
    """Quaternion (x, y, z, w) of the shortest rotation from +z to unit d."""
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z, d)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    angle = math.atan2(s, float(np.dot(z, d)))
    axis = axis / s * math.sin(angle / 2)
    return (float(axis[0]), float(axis[1]), float(axis[2]), math.cos(angle / 2))


class StemDetector(Node):
    def __init__(self):
        super().__init__('stem_detector')
        self.declare_parameter('image_topic', '/wrist_camera/image')
        self.declare_parameter('depth_topic', '/wrist_camera/depth_image')
        self.declare_parameter('info_topic', '/wrist_camera/camera_info')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('fruit_radius', 0.035)   # [m]
        self.declare_parameter('stem_radius', 0.003)    # [m]
        self.declare_parameter('grasp_above', 0.02)     # above the fruit top [m]
        self.declare_parameter('stem_height', 0.08)     # stem searched up to this [m]
        self.declare_parameter('region_radius', 0.05)   # sideways from the center [m]
        self.declare_parameter('min_area', 40)          # fruit blob [pixels]
        self.declare_parameter('min_stem_points', 30)
        self.declare_parameter('min_span', 0.02)        # along the stem [m]
        self.declare_parameter('max_rms', 0.003)        # off the fitted line [m]
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.K = None
        self.last_key = None
        p = self.get_parameter
        self.create_subscription(CameraInfo, p('info_topic').value, self.on_info, 1)
        self.sync = TimeSynchronizer(
            [Subscriber(self, Image, p('image_topic').value),
             Subscriber(self, Image, p('depth_topic').value)], 5)
        self.sync.registerCallback(self.on_images)
        self.pub = self.create_publisher(PoseStamped, '~/grasp', 1)
        self.image_pub = self.create_publisher(Image, '~/image', 1)

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def camera_pose(self, frame):
        try:
            t = self.tf.lookup_transform(self.get_parameter('target_frame').value, frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f'tf: {exc}', throttle_duration_sec=5.0)
            return None
        tr = t.transform
        return quat_to_matrix(tr.rotation), np.array(
            [tr.translation.x, tr.translation.y, tr.translation.z])

    def rays(self, us, vs):
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        return np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones(len(us))])

    def on_images(self, msg, depth_msg):
        if self.K is None:
            return
        cam = self.camera_pose(msg.header.frame_id)
        if cam is None:
            return
        rot, trans = cam
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        shown = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        valid = np.isfinite(depth) & (depth > 0)

        # 1. Nearest ripe fruit: the largest red blob.
        red = np.zeros(hsv.shape[:2], bool)
        for lo, hi in CLASSES['ripe']['hue']:
            red |= (hsv[..., 0] >= lo) & (hsv[..., 0] <= hi)
        red &= (hsv[..., 1] >= MIN_SATURATION) & (hsv[..., 2] >= MIN_VALUE)
        red = cv2.morphologyEx(red.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(red)
        if count < 2:
            return
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[k, cv2.CC_STAT_AREA] < p('min_area'):
            return
        vs, us = np.nonzero((labels == k) & valid)
        if len(us) < p('min_area') // 2:
            return
        dist = depth[vs, us] * np.linalg.norm(self.rays(us, vs), axis=0)
        u, v = centroids[k]
        ray = self.rays(np.array([u]), np.array([v]))[:, 0]
        radius = p('fruit_radius')
        center = rot @ (ray / np.linalg.norm(ray) * (np.percentile(dist, 5) + radius)) + trans
        x, y, w, h = stats[k, :4]
        cv2.rectangle(shown, (int(x), int(y)), (int(x + w), int(y + h)), (255, 0, 255), 1)

        # 2. Stem pixels to 3D, moved one stem radius behind the surface.
        stem = ((hsv[..., 0] >= STEM_HUE[0]) & (hsv[..., 0] <= STEM_HUE[1])
                & (hsv[..., 1] >= STEM_SATURATION[0]) & (hsv[..., 1] <= STEM_SATURATION[1])
                & (hsv[..., 2] >= MIN_VALUE) & valid)
        vs, us = np.nonzero(stem)
        if len(us) == 0:
            return
        r = self.rays(us, vs)
        pts_cam = r * depth[vs, us]
        pts_cam += r / np.linalg.norm(r, axis=0) * p('stem_radius')
        pts = (rot @ pts_cam).T + trans

        # 3. Points above this fruit.
        top = center[2] + radius
        side = np.linalg.norm(pts[:, :2] - center[:2], axis=1)
        keep = (side <= p('region_radius')) & (pts[:, 2] >= top) & (pts[:, 2] <= top + p('stem_height'))
        if keep.sum() < p('min_stem_points'):
            self.get_logger().warn(f'only {int(keep.sum())} stem points above the fruit',
                                   throttle_duration_sec=5.0)
            return
        pts = pts[keep]
        for uu, vv in zip(us[keep][::3], vs[keep][::3]):
            shown[vv, uu] = (0, 255, 255)

        # 4. Principal axis through the points, pointing up.
        mean = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - mean, full_matrices=False)
        d = vt[0] if vt[0][2] >= 0 else -vt[0]
        if d[2] < 0.3:      # more than ~70° from vertical: not a stem
            return
        residual = np.linalg.norm(np.cross(pts - mean, d), axis=1)
        rms = math.sqrt(np.mean(residual ** 2))
        along = (pts - mean) @ d
        span = float(along.max() - along.min())
        if span < p('min_span') or rms > p('max_rms'):
            self.get_logger().warn(
                f'stem fit rejected: {len(pts)} points spanning {100 * span:.1f} cm, '
                f'rms {1000 * rms:.1f} mm', throttle_duration_sec=5.0)
            return
        grasp_z = top + p('grasp_above')
        grasp = mean + d * (grasp_z - mean[2]) / d[2]

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = p('target_frame')
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, grasp)
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = quat_from_z(d)
        self.pub.publish(out)
        img = self.bridge.cv2_to_imgmsg(shown, 'bgr8')
        img.header = msg.header
        self.image_pub.publish(img)
        tilt = math.degrees(math.acos(min(1.0, float(d[2]))))
        key = tuple(np.round(grasp / 0.002).astype(int)) + (round(tilt),)
        if key != self.last_key:
            self.last_key = key
            self.get_logger().info(
                'grasp point ' + ' '.join(f'{c:+.3f}' for c in grasp)
                + f', stem tilt {tilt:.1f}° toward ({d[0]:+.2f} {d[1]:+.2f}), '
                f'{len(pts)} points spanning {100 * span:.1f} cm (rms {1000 * rms:.1f} mm), '
                'fruit center ' + ' '.join(f'{c:+.3f}' for c in center))


def main():
    rclpy.init()
    node = StemDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
