"""Find ripe and unripe fruit in an RGB-D image and publish their 3D centers.

For each color image the node:

1. Converts it to HSV and keeps the pixels whose hue and saturation look
   like a ripe (red) or an unripe (yellow-green) fruit.
2. Splits each mask into connected blobs and drops small ones (noise or
   fruit mostly hidden behind something).
3. Reads the depth image at the blob. The nearest part of a sphere seen by
   the camera is one radius in front of its center, so the center lies one
   radius beyond the nearest depth along the ray through the blob centroid.
4. Transforms the center into the target frame (world) with tf, and drops
   anything within tool_exclusion of the tool: a red payload carried by
   the arm, or the fruit that is being picked right now.

Ripe fruit go to ~/ripe and unripe fruit to ~/unripe as PoseArray, and an
image with the blobs marked goes to ~/image.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# OpenCV hue runs 0-179. Ripe red wraps around 0.
CLASSES = {
    'ripe': {'hue': [(0, 8), (172, 179)], 'color': (255, 0, 255)},
    'unripe': {'hue': [(30, 55)], 'color': (0, 255, 255)},
}
MIN_SATURATION = 120
MIN_VALUE = 40


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FruitDetector(Node):
    def __init__(self):
        super().__init__('fruit_detector')
        self.declare_parameter('image_topic', '/field_camera/image')
        self.declare_parameter('depth_topic', '/field_camera/depth_image')
        self.declare_parameter('info_topic', '/field_camera/camera_info')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('fruit_radius', 0.035)   # [m]
        self.declare_parameter('min_area', 40)          # [pixels]
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('tool_exclusion', 0.08)  # [m]
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.K = None
        self.last_summary = None
        p = self.get_parameter
        self.create_subscription(CameraInfo, p('info_topic').value, self.on_info, 1)
        # Gazebo stamps the color and depth images of one frame identically,
        # so pair them by exact stamp, whichever arrives first.
        self.sync = TimeSynchronizer(
            [Subscriber(self, Image, p('image_topic').value),
             Subscriber(self, Image, p('depth_topic').value)], 5)
        self.sync.registerCallback(self.on_images)
        self.pubs = {name: self.create_publisher(PoseArray, f'~/{name}', 1)
                     for name in CLASSES}
        self.image_pub = self.create_publisher(Image, '~/image', 1)

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def lookup(self, target, source):
        # Latest transform, without waiting: the camera is fixed, and the
        # tool position is only used for the coarse exclusion check.
        try:
            t = self.tf.lookup_transform(target, source, Time())
        except TransformException as exc:
            self.get_logger().warn(f'tf {target} <- {source}: {exc}', throttle_duration_sec=5.0)
            return None
        tr = t.transform
        return quat_to_matrix(tr.rotation), np.array(
            [tr.translation.x, tr.translation.y, tr.translation.z])

    def on_images(self, msg, depth_msg):
        if self.K is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        target = self.get_parameter('target_frame').value
        cam = self.lookup(target, msg.header.frame_id)
        tool = self.lookup(target, self.get_parameter('tool_frame').value)
        if cam is None:
            return
        tool_pos = None if tool is None else tool[1]

        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        sat_ok = (hsv[..., 1] >= MIN_SATURATION) & (hsv[..., 2] >= MIN_VALUE)
        shown = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        summary, key = [], []
        for name, cls in CLASSES.items():
            mask = np.zeros(hsv.shape[:2], bool)
            for lo, hi in cls['hue']:
                mask |= (hsv[..., 0] >= lo) & (hsv[..., 0] <= hi)
            mask = (mask & sat_ok).astype(np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            centers = self.find(mask, depth, cam, tool_pos, shown, cls['color'])
            out = PoseArray()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = target
            for c in centers:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = map(float, c)
                pose.orientation.w = 1.0
                out.poses.append(pose)
            self.pubs[name].publish(out)
            key.append(tuple(tuple(np.round(np.array(c) / 0.005).astype(int)) for c in centers))
            summary.append(f'{name} {len(centers)}: ' + ', '.join(
                '(' + ' '.join(f'{v:+.3f}' for v in c) + ')' for c in centers))
        out_img = self.bridge.cv2_to_imgmsg(shown, 'bgr8')
        out_img.header = msg.header
        self.image_pub.publish(out_img)
        # Log only when the count changes or a fruit moves by ~5 mm.
        if key != self.last_summary:
            self.last_summary = key
            self.get_logger().info(f'fruit in {target}: ' + ' | '.join(summary))

    def find(self, mask, depth, cam, tool_pos, shown, color):
        """Return 3D centers of the blobs in mask, in the target frame."""
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        radius = self.get_parameter('fruit_radius').value
        min_area = self.get_parameter('min_area').value
        exclusion = self.get_parameter('tool_exclusion').value
        rot, trans = cam
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
        centers = []
        for k in range(1, count):
            if stats[k, cv2.CC_STAT_AREA] < min_area:
                continue
            vs, us = np.nonzero(labels == k)
            z = depth[vs, us]
            ok = np.isfinite(z) & (z > 0)
            if ok.sum() < min_area // 2:
                continue
            # Distance along each pixel's ray, then the nearest few percent.
            rays = np.stack([(us[ok] - cx) / fx, (vs[ok] - cy) / fy, np.ones(ok.sum())])
            dist = z[ok] * np.linalg.norm(rays, axis=0)
            nearest = np.percentile(dist, 5)
            u, v = centroids[k]
            ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            center_cam = ray / np.linalg.norm(ray) * (nearest + radius)
            center = rot @ center_cam + trans
            if tool_pos is not None and np.linalg.norm(center - tool_pos) < exclusion:
                continue
            centers.append(center)
            x, y, w, h = stats[k, :4]
            cv2.rectangle(shown, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
            cv2.putText(shown, f'{center[0]:.2f},{center[1]:.2f},{center[2]:.2f}',
                        (int(x), max(12, int(y) - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, color, 1)
        return centers


def main():
    rclpy.init()
    node = FruitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
