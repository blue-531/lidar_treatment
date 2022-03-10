#include <ros/ros.h>
#include <ros/console.h>

#include <time.h> 
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>






#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
using namespace std;

string pointCloudTopicTop = "vlp_t/velodyne_points";
string pointCloudTopicLeft = "vlp_l/velodyne_points";
string pointCloudTopicRight = "vlp_r/velodyne_points";
typedef pcl::PointXYZI PointType;

float xL = -0.00650766;
float yL = 0.499486;
float zL = -0.218035;
float rollL = -14.122;
float pitchL = -0.89415;
float yawL = 0.262042;

float xT = 0.0;
float yT = 0.0;
float zT = 0.0 ;
float rollT = 0.0;
float pitchT = 0.0;
float yawT = 0.0;

float xR = 0.0;
float yR =  -0.52198;
float zR = -0.196542;
float rollR =  14.521;
float pitchR = -0.0367194;
float yawR =  0.301982;
