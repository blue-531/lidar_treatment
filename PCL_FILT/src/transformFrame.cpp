#include "utility.h"

using namespace std;
class lidarTransform {
private:
  ros::NodeHandle nh;

public:

  ros::Subscriber subTop;
  ros::Subscriber subLeft;
  ros::Subscriber subRight;
  ros::Publisher pubTop;
  ros::Publisher pubLeft;
  ros::Publisher pubRight;
  sensor_msgs::PointCloud2 pubTsensor;
  sensor_msgs::PointCloud2 pubLsensor;
  sensor_msgs::PointCloud2 pubRsensor;
  Eigen::Affine3f transLeftToTop;
  Eigen::Affine3f transRightToTop;
  Eigen::Matrix4f M_for_l;
  Eigen::Matrix4f M_for_r;

  ros::Publisher pubPointCloud;
  ros::Publisher pubCombine;

    lidarTransform() {
      // Transformation matrix of frames
      Eigen::Affine3f transLeft = pcl::getTransformation(xL, yL, zL, rollL*M_PI/180, pitchL*M_PI/180, yawL*M_PI/180);
      Eigen::Affine3f transRight = pcl::getTransformation(xR, yR, zR, rollR*M_PI/180, pitchR*M_PI/180, yawR*M_PI/180);

      transLeftToTop = transLeft;
      transRightToTop = transRight;

      subTop = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopicTop, 5, &lidarTransform::topTransformCB, this, ros::TransportHints().tcpNoDelay());
      subLeft = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopicLeft, 5, &lidarTransform::leftTransformCB, this, ros::TransportHints().tcpNoDelay());
      subRight = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopicRight, 5, &lidarTransform::rightTransformCB, this, ros::TransportHints().tcpNoDelay());

      pubTop = nh.advertise<sensor_msgs::PointCloud2> ("transformedTop", 10);
      pubLeft = nh.advertise<sensor_msgs::PointCloud2> ("transformedLeft", 10);
      pubRight = nh.advertise<sensor_msgs::PointCloud2> ("transformedRight", 10);

    }

    void topTransformCB(const sensor_msgs::PointCloud2ConstPtr& topLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclTop (new pcl::PointCloud<pcl::PointXYZI> ());

      pcl::fromROSMsg(*topLaserMsg, *pclTop);
      pcl::toROSMsg(*pclTop, pubTsensor);
      pubTop.publish(pubTsensor);
    }

    void leftTransformCB(const sensor_msgs::PointCloud2ConstPtr& leftLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclLeft (new pcl::PointCloud<pcl::PointXYZI> ());
      // sensor_msg -> pcl pointcloud
      pcl::fromROSMsg(*leftLaserMsg, *pclLeft);

      // Transformation
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

      M_for_l= transLeftToTop.matrix();
      pcl::transformPointCloud (*pclLeft, *transformed_cloud, M_for_l);

      //pcl::transformPointCloud (*pclLeft, *transformed_cloud, transLeftToTop);
      pcl::toROSMsg(*transformed_cloud, pubLsensor);

      pubLsensor.header.frame_id = "top_velodyne";
      pubLeft.publish(pubLsensor);
    }

    void rightTransformCB(const sensor_msgs::PointCloud2ConstPtr& rightLaserMsg)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pclRight (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::fromROSMsg(*rightLaserMsg, *pclRight);

      M_for_r= transRightToTop.matrix();
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());

      pcl::transformPointCloud (*pclRight, *transformed_cloud, M_for_r);
      pcl::toROSMsg(*transformed_cloud, pubRsensor);
      pubRsensor.header.frame_id = "top_velodyne";
      pubRight.publish(pubRsensor);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_node");

    lidarTransform LT;

    ROS_INFO("\033[1;32m----> [AGV Global Module] : Image Projection / MIDDLE LIDAR\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
