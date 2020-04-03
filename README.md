# GL-3 Object Detector
- This ROS package provides 2D object detection using GL-3 LiDAR.
- GL-3 LiDAR specification: FOV:180 degree, max distance: 30m, resolution: 0.1 degree.
- The object_detector_node publishes 2d lidar raw dadta and object detection data. An object is formed as a "group".
- A "group_vector" consists of groups.

# Test Environment
- ROS Melodic Morenia & ROS Kinetic Kame.
- Ubuntu 16.04 & Ubuntu 18.04.
- x86_64 (PC), aarch64 (Jetson series).

# Guide
- Installation
~~~ 
cd ${ROS workspace}/src
git clone https://github.com/sos-lab-co-ltd/object_detector.git
cd ${ROS workspace}
catkin_make -DCMAKE_BUILD_TYPE=Release
~~~
${ROS workspace} is your ROS workspace directory. ex) catkin_ws.

- Set permission of USB Port
~~~
sudo chmod a+rw /dev/ttyUSB0
~~~
or
~~~
sudo usermod -a -G dialout $USER
~~~
and reboot.

- Run Object Detector Node
~~~
cd ${ROS workspace}/src/object_detector/launch
roslaunch object_detector.launch
~~~

- Run Object Detector Node With RViz
~~~
roslaunch view_object_detector.launch
~~~

# Published Topics
- **scan** (sensor_msgs/LaserScan): publishes scan topic from the laser.
- **obj_text** (visualization_msgs::Marker): displays x and y position of detected object.
- **obj_box** (visualization_msgs::Marker): displays the detected object using bounding box.
- **obj** (sensor_msgs::PointCloud2): publishes the whole detected objects in pointcloud.

# Parameter Explanation of object_detector.launch
- Name and port customization
  - **serial_port_name** (std::string): serial port name. (default: "/dev/ttyUSB0")
  - **frame_id** (std::string): frame id name. (default: "laser_frame")
  - **pub_topicname_lidar** (std::string): lidar publish topic name. (default: "scan")
  - **pub_topicname_obj** (std::string): object detection pointcloud publish topic name. (default: "obj")
  
- Rviz drawing related
  - **ref_save_time** (double): time period that saves initial reference position. Used to constrain the detection boundary and remove walls. If set zero, every detected object is displayed. (default: 2 seconds)
  - **rviz_scale** (double): adjust object text display size. (default: 5)
  - **group_box_size_threshold** (double) : compares the distance between new object and initial reference points such as walls, ignore the object below this threshold parameter. (default: 0.3 meter)
  - **loop_hz** (int): adjust detection rate. (default: 40 Hz)
  - **rviz_pub_hz** (int): adjust rviz pulbish rate. (default: 20 Hz)

- Object detection filter related: Gathering points to form a single object by comparing distance between points.
  - **max_distance** (double): reference distance for max_thr_dist. (default: 30 meter)
  - **min_distance** (double): reference distance for min_thr_dist. (default: 0.1 meter)
  - **min_thr_dist** (double): compare the distance between points, add a point to form an object above this value. (default: 0.06 meter)
  - **max_thr_dist** (double): compare the distance between points, add a point to form an object under this value.  (default: 0.2 meter)
  - ex) when around 0.1 meter (==min_distance), the points under 0.06 meter (==min_thr_dist) is ignored. Also, when around 30 meter (==max_distance), the points under 0.2 meter (==max_thr_dist) is ignored. The values in between are automatically adjusted linearly.

- Object detection filter related: Filtering noise by point numbers of an object 
  - **filt_max_dist** (double): reference distance for filt_max_point_num. (default: 20 meter)
  - **filt_min_dist** (double): reference distance for filt_min_point_num. (default: 1 meter)
  - **filt_max_point_num** (int): object point number under this is ignored. (default: 1 point)
  - **filt_min_point_num** (int): object point number under this is ignored. (default: 10 points)
  - ex) when around 1 meter (==filt_min_distance), the object under 10 points (==filt_min_point_num) is ignored. Also, when around 20 meter (==filt_max_distance), the object under 1 point (==filt_max_point) is ignored. The values in between are automatically adjusted linearly.
