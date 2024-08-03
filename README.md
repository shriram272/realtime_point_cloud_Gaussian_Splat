This project is part of a larger project which is focussed on implemting Gaussian Splatting SLAM in mobile robots.

This part is for image to point cloud in realtime using a turtlebot in ROS2. The image is taken from the image topic while DepthAnything is used to provide a depth map. Both are then matched and filtered to generate a realtime point cloud.
This approach was tested to improve the exixting Gaussian Splatting which uses SFM tools like Colmap etc. It was tested to see if depth map and images can be used to provide a Gaussian Splat.

Here's a brief description of the important logics used in the code -

1.The sigmoid function scales depth values. It maps input values (depth) into a range that can be more suitable for point cloud processing. Here, a controls the steepness of the sigmoid curve.
There are two implementations - 
Instead of hardcoding for more robust estimation the steepness parameter was estimated using the 2D Lidar. The 2D Lidar scan provides the distance in a specific plane. The coordinates were converted from polar to cartesian and frame transformations were used and these points were superimposed on 2D image to get depth of corresponding points with help of camera intrinsic matrix.
This helps in calculation of maximum and minimum absolute distances.
Steepness Parameter (a) - 
The parameter a is determined to ensure the depth values are scaled appropriately based on the distance_ratio, farthest_distance, and nearest_distance. The goal is to find aa such that the ratio of the farthest distance to the nearest distance is consistent with the observed ratio in your data.
You need a to match the ratio of the farthest distance to the nearest distance. The sigmoid function's output for the farthest distance should be close to 1, and for the nearest distance, it should be close to 0.

def equation(a):
    numerator = 1 + np.exp(-a * (self.farthest_distance - 0.5))
    denominator = 1 + np.exp(-a * (self.nearest_distance - 0.5))
    ratio = numerator / denominator
    return ratio
Use fsolve from scipy.optimize to find aa that satisfies the equation. The function attempts to find aa where the ratio matches the expected distance ratio:
    




2.Subscriptions: Listens to topics /camera/image_raw for RGB images and /depth_result for depth images.
Publisher: Publishes point cloud data to the /camera/point_cloud topic.
3.convert_image: Converts ROS image message data into a NumPy array. Adjusts data type based on encoding (16-bit or 8-bit).
4.convert_from_uvd: Converts (u, v, depth) coordinates to (x, y, z) in 3D space using intrinsic camera parameters.
5.depth_to_point_cloud: Converts the depth and RGB images into a point cloud. It scales the depth values using a sigmoid function and converts pixel coordinates to 3D points. The resulting point cloud is then created and populated with RGB values.
6.point_cloud_to_ros2: Converts the Open3D point cloud object to a ROS 2 PointCloud2 message. It prepares the header, defines point fields, and packs the point cloud data into the message format.
7.The final code is for converting a ply file generated from photogammetry or Gaussian Splatting in Rviz.


RESULTS-



![WhatsApp Image 2024-08-03 at 16 45 58](https://github.com/user-attachments/assets/6a7255d6-0450-4d20-903d-132640ea197f)
![WhatsApp Image 2024-08-03 at 16 46 00](https://github.com/user-attachments/assets/b58ee982-ac5d-4f03-8bff-c18e0a5fa9fe)
![WhatsApp Image 2024-08-03 at 16 46 01](https://github.com/user-attachments/assets/bca39b54-73a9-46a8-86ab-5b482af79c40)
![WhatsApp Image 2024-08-03 at 16 46 02](https://github.com/user-attachments/assets/7b7b80b1-aa5c-4fb7-8966-ba75aca5b02b)
![WhatsApp Image 2024-07-28 at 16 07 52](https://github.com/user-attachments/assets/cf486341-c929-491e-ba0a-578dcc89d6f6)



VIDEOS-



https://github.com/user-attachments/assets/3860a737-ba40-4147-842f-5e9508a00446




https://github.com/user-attachments/assets/00cc50bc-c161-4e54-a628-2ea1c743d0fd






