#!/usr/bin/env python3
import os
import rclpy
import cv2
import tf2_ros
import numpy as np
import pyrealsense2 as rs

from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


class YOLObjectDetect(Node):

    def __init__(self):
        super().__init__('object_detect_yolo')

        # ---------- Parameters ----------
        default_weights = os.path.expanduser(
            '/home/mtrn/mtrn4231_jakos/ros2_system/src/perception_module/best.pt'
        )
        self.declare_parameter('yolo_weights', default_weights)
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('target_class_name', '')   # empty = accept all classes
        self.declare_parameter('debug_view', True)

        # ---------- Height / weight estimation tuning ----------
        # how far below the top of the bbox (in pixels) to sample "top" depth
        # (to avoid the shiny metal knob)
        self.declare_parameter('top_offset_px', 8)
        # how far below the bottom of the bbox to sample the table depth
        self.declare_parameter('table_offset_px', 10)
        # half-size of the square patch used to median-filter the depth
        self.declare_parameter('patch_half_size', 3)

        self.top_offset_px = int(self.get_parameter('top_offset_px').value)
        self.table_offset_px = int(self.get_parameter('table_offset_px').value)
        self.patch_half_size = int(self.get_parameter('patch_half_size').value)

        weights_path = self.get_parameter('yolo_weights').get_parameter_value().string_value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.target_class = self.get_parameter('target_class_name').get_parameter_value().string_value
        self.debug_view = bool(self.get_parameter('debug_view').value)

        # ---------- Depth camera subscriptions ----------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.arm_image_callback,
            10,
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.arm_depth_callback,
            10,
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.arm_image_depth_info_callback,
            10,
        )

        self.intrinsics = None
        self.depth_image = None
        self.cv_image = None
        self.cv_bridge = CvBridge()

        # ---------- TF ----------
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- Publishers for other nodes ----------
        # 3D position in base_link (meters)
        self.coord_pub = self.create_publisher(
            PointStamped,
            'object_coords',
            10
        )
        # Metadata: class, conf, estimated weight as a string
        self.info_pub = self.create_publisher(
            String,
            'object_weight',
            10
        )

        # ---------- Load YOLO ----------
        self.get_logger().info(f'Loading YOLO weights: {weights_path}')
        self.model = YOLO(weights_path)
        self.names = self.model.names
        self.get_logger().info(f'YOLO classes: {self.names}')

        # ---------- Timer ----------
        self.routine_timer = self.create_timer(0.05, self.routine_callback)

        self.get_logger().info('YOLOObjectDetect node ready.')

    # ----------------- Camera info -> RealSense intrinsics -----------------
    def arm_image_depth_info_callback(self, cameraInfo: CameraInfo):
        try:
            if self.intrinsics:
                return
            intr = rs.intrinsics()
            intr.width = cameraInfo.width
            intr.height = cameraInfo.height
            intr.ppx = cameraInfo.k[2]
            intr.ppy = cameraInfo.k[5]
            intr.fx = cameraInfo.k[0]
            intr.fy = cameraInfo.k[4]

            if cameraInfo.distortion_model == 'plumb_bob':
                intr.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                intr.model = rs.distortion.kannala_brandt4
            else:
                intr.model = rs.distortion.none

            intr.coeffs = [c for c in cameraInfo.d]
            self.intrinsics = intr
            self.get_logger().info('Camera intrinsics set.')
        except CvBridgeError as e:
            self.get_logger().error(f'Intrinsics error: {e}')

    # ----------------- Image callbacks -----------------
    def arm_image_callback(self, msg: Image):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

    def arm_depth_callback(self, msg: Image):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error in arm_depth_callback: {str(e)}")
    
    def get_view_ray(self, u, v):
        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        cx = self.intrinsics.ppx
        cy = self.intrinsics.ppy

        # ray in optical frame (unnormalized)
        x = (u - cx) / fx
        y = (v - cy) / fy
        z = 1.0

        # normalize
        ray = np.array([x, y, z], dtype=float)
        ray = ray / np.linalg.norm(ray)
        return ray

    # ----------------- Pixel -> 3D in camera frame -----------------
    def pixel_2_global(self, u: int, v: int):
        if self.depth_image is None or self.intrinsics is None:
            return None

        h, w = self.depth_image.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        #self.get_logger().info(f"depth dtype={self.depth_image.dtype}, sample={self.depth_image[v, u]}")  # depth value check in mm or m
        
        #depth_val_m = float(self.depth_image[v, u]) * 0.001  # depth in mm -> m
        
        # Depth smoothing window
        kernel = 5 # 3 or 5 are typical
        v1 = max(0, v - kernel//2)
        v2 = min(self.depth_image.shape[0], v + kernel//2 + 1)
        u1 = max(0, u - kernel//2)
        u2 = min(self.depth_image.shape[1], u + kernel//2 + 1)

        window = self.depth_image[v1:v2, u1:u2]
        depth_val_m = float(np.median(window)) * 0.001
        
        # depth correction (meters)
        #depth_correction = 0  # 10 mm, adjust as needed

        # compute view ray
        #ray = self.get_view_ray(u, v)  # unit vector in OPTICAL frame

        # apply depth correction along ray
        #depth_val_m_corrected = depth_val_m - depth_correction
        #depth_val_m_corrected = max(depth_val_m_corrected, 0.0)

        if depth_val_m <= 0.0 or np.isnan(depth_val_m):
            return None

        # Optical frame coordinates from RealSense
        Xo, Yo, Zo = rs.rs2_deproject_pixel_to_point(self.intrinsics, (u, v), depth_val_m)

        # Convert OPTICAL → camera_link frame
        Xc = Zo
        Yc = -Xo
        Zc = -Yo

        return [Xc, Yc, Zc]

    def _median_depth_in_patch(self, u, v):
        """
        Get a robust median depth (in meters) around pixel (u, v)
        using a small square patch. Returns None if no valid depth.
        """
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]
        ps = self.patch_half_size

        u_min = max(0, u - ps)
        u_max = min(w - 1, u + ps)
        v_min = max(0, v - ps)
        v_max = min(h - 1, v + ps)

        patch = self.depth_image[v_min:v_max + 1, u_min:u_max + 1].astype(np.float32)

        # RealSense depth is in mm (uint16); convert to meters and ignore zeros
        flat = patch.reshape(-1)
        valid = flat[flat > 0.0]
        if valid.size == 0:
            return None

        depth_m = np.median(valid) * 0.001  # mm -> m
        return depth_m

    def estimate_height_m(self, x1, y1, x2, y2):
        """
        Estimate cylinder height in meters using depth.
        We sample one point slightly below the top of the bbox for the object,
        and one point below the bottom of the bbox for the table.

        Returns:
            None if estimation fails
            OR (height_m, (u_top, v_top), (u_table, v_table))
        """
        if self.depth_image is None:
            return None

        h, w = self.depth_image.shape[:2]

        # horizontal centre of the object
        u_center = int((x1 + x2) / 2.0)

        # pixel for "top" of object: just below top edge of bbox (avoid shiny knob)
        v_top = int(y1) + self.top_offset_px
        v_top = max(0, min(h - 1, v_top))

        # pixel for table: a bit below the bottom of the bbox
        v_table = int(y2) + self.table_offset_px
        v_table = max(0, min(h - 1, v_table))

        depth_top = self._median_depth_in_patch(u_center, v_top)
        depth_table = self._median_depth_in_patch(u_center, v_table)

        if depth_top is None or depth_table is None:
            self.get_logger().warn("Height estimation: invalid depth (top or table)")
            return None

        # Table is further from the camera than the top of the object
        height_m = depth_table - depth_top

        # sanity check
        if height_m <= 0.0 or height_m > 0.5:  # 0.5 m upper bound just to catch nonsense
            self.get_logger().warn(
                f"Height estimation suspicious: {height_m:.3f} m "
                f"(top_depth={depth_top:.3f} m, table_depth={depth_table:.3f} m)"
            )
            return None

        return height_m, (u_center, v_top), (u_center, v_table)

    def estimate_weight_from_height(self, height_m):
        """
        Map a measured height (in meters) to a weight label.
        >>> YOU MUST TUNE THESE THRESHOLDS BASED ON YOUR ACTUAL WEIGHTS. <<<
        """
        # TODO: replace these with real measured heights for your weights
        if height_m > 0.040:      # > 4.0 cm
            return 500
        elif height_m > 0.030:    # 3–4 cm
            return 200
        elif height_m > 0.025:    # 2.5–3 cm
            return 100
        elif height_m > 0.020:    # 2–2.5 cm
            return 50
        elif height_m > 0.015:    # 2–1.5 cm
            return 20
        else:
            return 10

    def refine_center_with_circle(self, img, x1, y1, x2, y2):
        """
        Refine the object centre by detecting the red circular top inside the YOLO bbox.
        Returns (u, v, r_px) in image coordinates.
        Falls back to bbox centre if detection fails.
        """
        h, w, _ = img.shape
        x1i = max(0, int(x1))
        y1i = max(0, int(y1))
        x2i = min(w - 1, int(x2))
        y2i = min(h - 1, int(y2))

        if x2i <= x1i or y2i <= y1i:
            # degenerate box – fallback
            u = int((x1 + x2) / 2.0)
            v = int((y1 + y2) / 2.0)
            return u, v, 0

        roi = img[y1i:y2i, x1i:x2i]

        # Convert ROI to HSV for robust red detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Two ranges for red (wraps around 180)
        lower_red1 = np.array([0, 50, 50])  # Adjusted S/V range
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up the mask a bit
        mask = cv2.medianBlur(mask, 5)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            # if we fail to find a contour, fallback to bbox centre
            u = int((x1 + x2) / 2.0)
            v = int((y1 + y2) / 2.0)
            return u, v, 0

        # Find the largest red contour (should be the weight top)
        c = max(contours, key=cv2.contourArea)
        (cx, cy), r = cv2.minEnclosingCircle(c)

        # Optional: filter out contours that are too small
        if r < 5:  # Minimum radius (adjust depending on your object's size)
            return int((x1 + x2) / 2.0), int((y1 + y2) / 2.0), 0

        # Convert ROI coordinates back to full-image coordinates
        u = int(x1i + cx)
        v = int(y1i + cy)
        r_px = int(r)

        # Apply an offset to move the centroid upwards
        vertical_offset = 10  # Move centroid 10 pixels up
        v = max(0, v - vertical_offset)  # Ensure that we don't go out of bounds

        return u, v, r_px

    # ----------------- Helper: quaternion -> rotation matrix -----------------
   # @staticmethod
   # def quaternion_to_rotation_matrix(qx, qy, qz, qw):
       # norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
       # if norm == 0.0:
         #   return np.eye(3)
       # x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
       # return np.array([
        #    [1 - 2*(y*y + z*z),     2*(x*y - z*w),         2*(x*z + y*w)],
        #    [2*(x*y + z*w),         1 - 2*(x*x + z*z),     2*(y*z - x*w)],
         #   [2*(x*z - y*w),         2*(y*z + x*w),         1 - 2*(x*x + y*y)]
       # ])

    # ----------------- Main routine: YOLO + depth -----------------
    def routine_callback(self):
        if self.cv_image is None or self.depth_image is None or self.intrinsics is None:
            return

        img = self.cv_image.copy()

        try:
            results = self.model.predict(img, conf=self.conf_thres, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            return

        if results.boxes is None or len(results.boxes) == 0:
            if self.debug_view:
                cv2.imshow("YOLO Detection", img)
                cv2.waitKey(1)
            return

        for i, box in enumerate(results.boxes):
            cls_id = int(box.cls[0])
            cls_name = self.names.get(cls_id, str(cls_id))
            conf = float(box.conf[0])

            if self.target_class and cls_name != self.target_class:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            #u = int((x1 + x2) / 2.0)
            #v = int((y1 + y2) / 2.0)

            # Refine centre using the circular top of the red weight
            u, v, r_px = self.refine_center_with_circle(img, x1, y1, x2, y2)
                
            # ---- Estimate cylinder height & weight from depth (before TF) ----
            height_result = self.estimate_height_m(x1, y1, x2, y2)
            weight_label = None
            top_px = None
            table_px = None

            if height_result is not None:
                height_m, top_px, table_px = height_result
                weight_label = self.estimate_weight_from_height(height_m)
                self.get_logger().info(
                    f"Estimated height: {height_m*1000.0:.1f} mm, "
                    f"estimated weight: {weight_label}"
                )

            cam_pt = self.pixel_2_global(u, v)
            if cam_pt is None:
                continue

            p_cam = np.array(cam_pt, dtype=float)

            # ---- Look up base_link <- camera_link transform from TF ----
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link",          # target
                    "camera_link",        # source
                    Time())               # latest available
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed: {ex}")
                continue

            #t = tf.transform.translation
            #q = tf.transform.rotation

            # ---------------- Apply camera offset ----------------
            # Offset is along camera X axis
            p_cam[0] += -0.038   # meters

            # Convert to PointStamped in camera frame
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "camera_link"
            pt.point.x = float(p_cam[0])
            pt.point.y = float(p_cam[1])
            pt.point.z = float(p_cam[2])

            try:
                #transform = self.tf_buffer.lookup_transform(
                    #"base_link",
                    #"camera_link",
                    #Time()
                #)
                pt_base = do_transform_point(pt, tf)

                x_b = pt_base.point.x
                y_b = pt_base.point.y
                z_b = pt_base.point.z

                # convert to mm for UR pendant
                x_mm = -x_b * 1000.0 - 20.0
                y_mm = -y_b * 1000.0 + 8.0
                z_b = z_b + 0.04
                z_mm = z_b * 1000.0

                self.get_logger().info(
                    f"[YOLO] {cls_name} ({conf:.2f})\n"
                    f"UR Base (mm): X={x_mm:.2f}, Y={y_mm:.2f}, Z={z_mm:.2f}"
                )

                # -------- Publish to other nodes --------
                # 1) 3D point in base_link (meters)
                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = "base_link"
                pt_msg.point.x = float(x_b)
                pt_msg.point.y = float(y_b)
                pt_msg.point.z = float(z_b)
                self.coord_pub.publish(pt_msg)

                # 2) Info string: class, conf, estimated weight
                info = String()
                # Example format: "red_object,0.87,200 g"
                if weight_label is not None:
                    info.data = f"{cls_name},{conf:.2f},{weight_label}"
                else:
                    info.data = f"{cls_name},{conf:.2f},unknown"
                self.info_pub.publish(info)

                # Publish TF in base frame
                t_out = TransformStamped()
                t_out.header.stamp = self.get_clock().now().to_msg()
                t_out.header.frame_id = "base_link"
                t_out.child_frame_id = f"yolo_object_{i}"
                t_out.transform.translation.x = float(x_b)
                t_out.transform.translation.y = float(y_b)
                t_out.transform.translation.z = float(z_b)
                t_out.transform.rotation.x = 0.0
                t_out.transform.rotation.y = 0.0
                t_out.transform.rotation.z = 0.0
                t_out.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t_out)

            except TransformException as ex:
                self.get_logger().warn(f"TF transform failed: {ex}")
                continue

            # ---- Draw BLUE bounding box + circle ----
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1) #Bounding Box
            cv2.circle(img, (u, v), 4, (255, 0, 0), -1) #Red centre dot

            # Draw circle showing the fitted top of the cylinder (if radius > 0)
            if r_px > 0:
                cv2.circle(img, (u, v), r_px, (0, 255, 0), 2)

            if weight_label is not None:
                label_text = f"{cls_name} {conf:.2f} {weight_label}"
            else:
                label_text = f"{cls_name} {conf:.2f}"

            cv2.putText(
                img,
                label_text,
                (int(x1), max(0, int(y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )#Object CLass - Text

        if self.debug_view:
            cv2.imshow("YOLO Detection", img)

            # Optional separate view just for the sampling pixels
            height_debug = np.zeros_like(img)
            for (px, label, color) in [
                (top_px, "TOP", (0, 0, 255)),
                (table_px, "TABLE", (255, 0, 255)),
            ]:
                if px is not None:
                    cv2.circle(height_debug, px, 5, color, -1)
                    cv2.putText(
                        height_debug,
                        label,
                        (px[0] + 5, px[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        1,
                    )

            cv2.imshow("Height Sampling Pixels", height_debug)
            key = cv2.waitKey(1) & 0xFF   # get key pressed (if any)

            if key == ord('q'):
                self.get_logger().info("q pressed, shutting down node.")
                rclpy.shutdown()
                cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = YOLObjectDetect()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
