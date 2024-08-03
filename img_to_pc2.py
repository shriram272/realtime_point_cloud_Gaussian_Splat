import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Float32
import numpy as np
import open3d as o3d
import struct
from scipy.optimize import fsolve

# Define sigmoid function for scaling depth values
def sigmoid(x, a=0.9):
    return 1 / (1 + np.exp(-a * (x - 0.5)))

class DepthToPointCloud(Node):
    def _init_(self):
        super()._init_('depth_to_point_cloud')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_result', self.depth_callback, 10)
        self.distance_ratio_sub = self.create_subscription(Float32, '/distance_ratio', self.distance_ratio_callback, 10)
        self.farthest_distance_sub = self.create_subscription(Float32, '/farthest_distance', self.farthest_distance_callback, 10)
        self.nearest_distance_sub = self.create_subscription(Float32, '/nearest_distance', self.nearest_distance_callback, 10)
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/camera/point_cloud', 10)
        
        self.distance_ratio = None
        self.farthest_distance = None
        self.nearest_distance = None
        self.rgb_image = None
        self.depth_image = None
        self.intrinsic_params = {
            'width': 1920,
            'height': 1080,
            'fx': 1696.802685832259,
            'fy': 1696.802685832259,
            'cx': 960.5,
            'cy': 540.5,
            'distortion': np.array([0, 0, 0, 0, 0])
        }

        # Flag to indicate whether point cloud has been published
        self.point_cloud_published = False

    def image_callback(self, msg):
        self.get_logger().info('RGB image received')
        self.rgb_image = self.convert_image(msg)
        self.try_generate_point_cloud()

    def depth_callback(self, msg):
        self.get_logger().info('Depth image received')
        self.depth_image = self.convert_image(msg)
        self.try_generate_point_cloud()

    def distance_ratio_callback(self, msg):
        self.distance_ratio = msg.data
        self.get_logger().info(f'Received distance ratio: {self.distance_ratio}')
        self.try_generate_point_cloud()

    def farthest_distance_callback(self, msg):
        self.farthest_distance = msg.data
        self.get_logger().info(f'Received farthest distance: {self.farthest_distance}')
        self.try_generate_point_cloud()

    def nearest_distance_callback(self, msg):
        self.nearest_distance = msg.data
        self.get_logger().info(f'Received nearest distance: {self.nearest_distance}')
        self.try_generate_point_cloud()

    def convert_image(self, msg):
        dtype = np.uint16 if msg.encoding == '16UC1' else np.uint8
        return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)

    def convert_from_uvd(self, u, v, d, cx, cy, fx, fy, scale=1):
        z = d * scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

    def compute_a(self):
        if None in [self.distance_ratio, self.farthest_distance, self.nearest_distance]:
            self.get_logger().warn('Insufficient data to compute a')
            return None

        def equation(a):
            numerator = 1 + np.exp(-a * (self.farthest_distance - 0.5))
            denominator = 1 + np.exp(-a * (self.nearest_distance - 0.5))
            ratio = numerator / denominator
            return ratio 

        initial_guess = 0.8
        a_solution, = fsolve(equation, initial_guess)
        self.get_logger().info(f'Computed steepness parameter a: {a_solution}')
        return a_solution

    def try_generate_point_cloud(self):
        if self.rgb_image is not None and self.depth_image is not None:
            self.get_logger().info('Attempting to generate point cloud')

            a = self.compute_a()
            if a is None:
                self.get_logger().warn('Failed to compute steepness parameter a')
                return

            point_cloud = self.depth_to_point_cloud(a)
            if point_cloud:
                # Visualize the point cloud in Open3D
                self.visualize_point_cloud(point_cloud)
                
                # Publish the point cloud to ROS 2 topic
                point_cloud_msg = self.point_cloud_to_ros2(point_cloud)
                self.point_cloud_pub.publish(point_cloud_msg)
                self.get_logger().info('Published point cloud message')
                self.point_cloud_published = False  # Reset the flag to allow further processing

    def depth_to_point_cloud(self, a):
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn('RGB or Depth image not available yet')
            return None

        height, width, _ = self.depth_image.shape
        fx = self.intrinsic_params['fx']
        fy = self.intrinsic_params['fy']
        cx = self.intrinsic_params['cx']
        cy = self.intrinsic_params['cy']

        # Apply sigmoid scaling to the depth image
        depth_image_scaled = sigmoid(1 - (self.depth_image.astype(np.float32) / 255.0), a) * 3.18

        points = []
        colors = []

        for v in range(height):
            for u in range(width):
                z = depth_image_scaled[v, u]
                if z > 0:
                    x, y, z = self.convert_from_uvd(u, v, z, cx, cy, fx, fy, 1)
                    points.append([x, y, z])
                    colors.append(self.rgb_image[v, u] / 255.0)

        if len(points) == 0:
            self.get_logger().warn('No valid points found in depth image')
            return None

        points = np.array(points)
        colors = np.array(colors)

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        self.get_logger().info(f'Generated point cloud with {len(points)} points')

        return point_cloud

    def visualize_point_cloud(self, point_cloud):
        o3d.visualization.draw_geometries([point_cloud])

    def point_cloud_to_ros2(self, point_cloud):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_rgb_frame'

        points = np.asarray(point_cloud.points)
        colors = np.asarray(point_cloud.colors)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []

        for i in range(points.shape[0]):
            x, y, z = points[i]
            r, g, b = colors[i]
            r = int(r * 255.0)
            g = int(g * 255.0)
            b = int(b * 255.0)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            cloud_data.append([x, y, z, rgb])

        cloud_data_flat = []
        for p in cloud_data:
            cloud_data_flat.extend(p)

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(points)
        point_cloud_msg.is_dense = False
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.fields = fields
        point_cloud_msg.point_step = 16
        point_cloud_msg.row_step = 16 * len(points)
        point_cloud_msg.data = struct.pack('f' * len(cloud_data_flat), *cloud_data_flat)

        self.get_logger().info('Converted point cloud to ROS2 message')

        return point_cloud_msg

def main(args=None):
    rclpy.init(args=args)
    depth_to_point_cloud = DepthToPointCloud()
    rclpy.spin(depth_to_point_cloud)
    rclpy.shutdown()

if _name_ == '_main_':
    main()