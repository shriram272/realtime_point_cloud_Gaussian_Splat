# import rospy
# import sensor_msgs.point_cloud2 as pc2
# import sensor_msgs.msg as sensor_msgs
# import std_msgs.msg as std_msgs

# import numpy as np
# import open3d as o3d
# import os

# class PCDPublisher:
#     def __init__(self):
#         rospy.init_node('pcd_publisher_node', anonymous=True)
#         rospy.loginfo("PCD Publisher Node Initialized")

#         # Specify the path to the point cloud file directly in the code.
#         pcd_path = '/home/shriram/19_06_2024.ply'
#         rospy.loginfo(f"Point cloud file path: {pcd_path}")

#         # Check if the file exists
#         assert os.path.exists(pcd_path), "File doesn't exist."
#         rospy.loginfo("File exists. Reading point cloud file.")

#         # Use Open3D to read point clouds and meshes.
#         pcd = o3d.io.read_point_cloud(pcd_path)
#         rospy.loginfo("Point cloud file read successfully.")

#         # Convert the point cloud to numpy arrays for points and colors.
#         self.points = np.asarray(pcd.points)
#         self.colors = np.asarray(pcd.colors)
#         rospy.loginfo(f"Points shape: {self.points.shape}")
#         rospy.loginfo(f"Colors shape: {self.colors.shape}")

#         # Create a publisher that publishes sensor_msgs.PointCloud2 to the topic 'pcd'.
#         self.pcd_publisher = rospy.Publisher('pcd', sensor_msgs.PointCloud2, queue_size=10)
#         self.timer = rospy.Timer(rospy.Duration(1/30.0), self.timer_callback)

#         # Rotation matrix for visualization purposes. Rotates the point cloud on each timer callback.
#         self.R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi/48])
#         rospy.loginfo("Publisher and timer initialized.")

#     def timer_callback(self, event):
#         rospy.loginfo("Timer callback triggered.")
#         # Rotate the point cloud for visualization purposes (if needed)
#         # self.points = self.points @ self.R
        
#         # Convert the numpy array into a sensor_msgs.PointCloud2 object
#         self.pcd = point_cloud(self.points, self.colors, 'map')
#         rospy.loginfo("Point cloud message created.")

#         # Publish the PointCloud2 object
#         self.pcd_publisher.publish(self.pcd)
#         rospy.loginfo("Point cloud message published.")

# def point_cloud(points, colors, parent_frame):
#     """ Creates a point cloud message.
#     Args:
#         points: Nx3 array of xyz positions.
#         colors: Nx3 array of rgb colors.
#         parent_frame: frame in which the point cloud is defined
#     Returns:
#         sensor_msgs/PointCloud2 message
#     """
#     rospy.loginfo("Creating point cloud message.")
    
#     # In a PointCloud2 message, the point cloud is stored as an byte array.
#     ros_dtype = sensor_msgs.PointField.FLOAT32
#     dtype = np.float32
#     itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

#     # Combine points and colors into a single Nx6 array
#     data = np.hstack([points, colors]).astype(dtype).tobytes()

#     # The fields specify what the bytes represents. The first 4 bytes represents the x-coordinate, the next 4 the y-coordinate, etc.
#     fields = [sensor_msgs.PointField(
#         name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         for i, n in enumerate(['x', 'y', 'z', 'r', 'g', 'b'])]

#     # The PointCloud2 message also has a header which specifies which coordinate frame it is represented in.
#     header = std_msgs.Header(frame_id=parent_frame)
#     header.stamp = rospy.Time.now()

#     rospy.loginfo("Point cloud message created successfully.")
#     return sensor_msgs.PointCloud2(
#         header=header,
#         height=1, 
#         width=points.shape[0],
#         is_dense=False,
#         is_bigendian=False,
#         fields=fields,
#         point_step=(itemsize * 6), # Every point consists of three float32s and three colors.
#         row_step=(itemsize * 6 * points.shape[0]), 
#         data=data
#     )

# def main():
#     rospy.loginfo("Starting PCD Publisher Node.")
#     # Boilerplate code.
#     pcd_publisher = PCDPublisher()
#     rospy.spin()
    
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
#     pcd_publisher.pcd_publisher.unregister()
#     rospy.loginfo("PCD Publisher Node stopped.")

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python

# import rospy
# import sensor_msgs.point_cloud2 as pc2
# import sensor_msgs.msg as sensor_msgs
# import std_msgs.msg as std_msgs

# import numpy as np
# import open3d as o3d
# import os

# class PCDPublisher:
#     def __init__(self):
#         rospy.init_node('pcd_publisher_node', anonymous=True)
#         rospy.loginfo("PCD Publisher Node Initialized")

#         # Specify the path to the point cloud file directly in the code.
#         pcd_path = '/home/shriram/pc3.ply'
#         rospy.loginfo(f"Point cloud file path: {pcd_path}")

#         # Check if the file exists
#         assert os.path.exists(pcd_path), "File doesn't exist."
#         rospy.loginfo("File exists. Reading point cloud file.")

#         # Use Open3D to read point clouds and meshes.
#         pcd = o3d.io.read_point_cloud(pcd_path)
#         rospy.loginfo("Point cloud file read successfully.")

#         # Convert the point cloud to numpy arrays for points and colors.
#         self.points = np.asarray(pcd.points)
#         self.colors = np.asarray(pcd.colors)
#         rospy.loginfo(f"Points shape: {self.points.shape}")
#         rospy.loginfo(f"Colors shape: {self.colors.shape}")

#         # Create a publisher that publishes sensor_msgs.PointCloud2 to the topic 'pcd'.
#         self.pcd_publisher = rospy.Publisher('pcd', sensor_msgs.PointCloud2, queue_size=10)
#         self.timer = rospy.Timer(rospy.Duration(1/30.0), self.timer_callback)

#         rospy.loginfo("Publisher and timer initialized.")

#     def timer_callback(self, event):
#         rospy.loginfo("Timer callback triggered.")
        
#         # Convert the numpy array into a sensor_msgs.PointCloud2 object
#         self.pcd = point_cloud(self.points, self.colors, 'world')
#         rospy.loginfo("Point cloud message created.")

#         # Publish the PointCloud2 object
#         self.pcd_publisher.publish(self.pcd)
#         rospy.loginfo("Point cloud message published.")

# def point_cloud(points, colors, parent_frame):
#     """ Creates a point cloud message.
#     Args:
#         points: Nx3 array of xyz positions.
#         colors: Nx3 array of rgb colors.
#         parent_frame: frame in which the point cloud is defined
#     Returns:
#         sensor_msgs/PointCloud2 message
#     """
#     rospy.loginfo("Creating point cloud message.")
    
#     # In a PointCloud2 message, the point cloud is stored as a byte array.
#     ros_dtype = sensor_msgs.PointField.FLOAT32
#     dtype = np.float32
#     itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

#     # Combine points and colors into a single Nx6 array
#     data = np.hstack([points, colors]).astype(dtype).tobytes()

#     # The fields specify what the bytes represent. The first 4 bytes represent the x-coordinate, the next 4 the y-coordinate, etc.
#     fields = [sensor_msgs.PointField(
#         name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         for i, n in enumerate(['x', 'y', 'z', 'r', 'g', 'b'])]

#     # The PointCloud2 message also has a header which specifies which coordinate frame it is represented in.
#     header = std_msgs.Header(frame_id=parent_frame)
#     header.stamp = rospy.Time.now()

#     rospy.loginfo("Point cloud message created successfully.")
#     return sensor_msgs.PointCloud2(
#         header=header,
#         height=1, 
#         width=points.shape[0],
#         is_dense=False,
#         is_bigendian=False,
#         fields=fields,
#         point_step=(itemsize * 6), # Every point consists of three float32s and three colors.
#         row_step=(itemsize * 6 * points.shape[0]), 
#         data=data
#     )

# def main():
#     rospy.loginfo("Starting PCD Publisher Node.")
#     # Boilerplate code.
#     pcd_publisher = PCDPublisher()
#     rospy.spin()
    
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
#     pcd_publisher.pcd_publisher.unregister()
#     rospy.loginfo("PCD Publisher Node stopped.")

# if __name__ == '__main__':
#     main()




#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d
import os

class PCDPublisher:
    def __init__(self):
        rospy.init_node('pcd_publisher_node', anonymous=True)
        # rospy.loginfo("PCD Publisher Node Initialized")

        # Specify the path to the point cloud file directly in the code.
        pcd_path = '/home/shriram/200.ply'
        # rospy.loginfo(f"Point cloud file path: {pcd_path}")

        # Check if the file exists
        assert os.path.exists(pcd_path), "File doesn't exist."
        # rospy.loginfo("File exists. Reading point cloud file.")

        # Use Open3D to read point clouds and meshes.
        pcd = o3d.io.read_point_cloud(pcd_path)
        # rospy.loginfo("Point cloud file read successfully.")

        # Convert the point cloud to numpy arrays for points and colors.
        self.points = np.asarray(pcd.points)
        if pcd.has_colors():
            self.colors = np.asarray(pcd.colors)
        else:
            # Assign a default color (e.g., white) if no color information is available.
            self.colors = np.ones((self.points.shape[0], 3))

        # rospy.loginfo(f"Points shape: {self.points.shape}")
        # rospy.loginfo(f"Colors shape: {self.colors.shape}")

        # Create a publisher that publishes sensor_msgs.PointCloud2 to the topic 'pcd'.
        self.pcd_publisher = rospy.Publisher('pcd', sensor_msgs.PointCloud2, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1/30.0), self.timer_callback)

        # rospy.loginfo("Publisher and timer initialized.")

    def timer_callback(self, event):
        # rospy.loginfo("Timer callback triggered.")
        
        # Convert the numpy array into a sensor_msgs.PointCloud2 object
        self.pcd = point_cloud(self.points, self.colors, 'map')
        # rospy.loginfo("Point cloud message created.")

        # Publish the PointCloud2 object
        self.pcd_publisher.publish(self.pcd)
        # rospy.loginfo("Point cloud message published.")

def point_cloud(points, colors, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions.
        colors: Nx3 array of rgb colors.
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    # rospy.loginfo("Creating point cloud message.")
    
    # In a PointCloud2 message, the point cloud is stored as a byte array.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    # Combine points and colors into a single Nx6 array
    data = np.hstack([points, colors]).astype(dtype).tobytes()

    # The fields specify what the bytes represent. The first 4 bytes represent the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(['x', 'y', 'z', 'r', 'g', 'b'])]

    # The PointCloud2 message also has a header which specifies which coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)
    header.stamp = rospy.Time.now()

    rospy.loginfo("Point cloud message created successfully.")
    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 6), # Every point consists of three float32s and three colors.
        row_step=(itemsize * 6 * points.shape[0]), 
        data=data
    )

def main():
    # rospy.loginfo("Starting PCD Publisher Node.")
    # Boilerplate code.
    pcd_publisher = PCDPublisher()
    rospy.spin()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    pcd_publisher.pcd_publisher.unregister()
    rospy.loginfo("PCD Publisher Node stopped.")

if __name__ == '__main__':
    main()
