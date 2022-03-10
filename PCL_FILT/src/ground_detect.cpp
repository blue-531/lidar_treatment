#include "utility.h"


using namespace std;

class findGround {
private:
  ros::NodeHandle nh;

public:

  ros::Subscriber subTotal;
  ros::Publisher pubGround;
  ros::Publisher pubObject;


    findGround() {

    }
    subTotal = nh.subscribe<sensor_msgs::PointCloud2>("total_lidar", 5, &findGround::detect_ground_points, this, ros::TransportHints().tcpNoDelay());

    pubGround = nh.advertise<sensor_msgs::PointCloud2> ("ground_point", 10);

    pubObject = nh.advertise<sensor_msgs::PointCloud2> ("object_point", 10);


    void detect_ground_points(const sensor_msgs::PointCloud2ConstPtr& total_msg)
    {
      clock_t time = clock();

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_total (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_plane (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      pcl::ExtractIndices<pcl::PointXYZI> extract;

      sensor_msgs::PointCloud2 pubGroundmsg;
      sensor_msgs::PointCloud2 pubObjectmsg;

      // filter by z axis only near floorr

      pcl::fromROSMsg(*total_msg, *pcl_total);
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud (pcl_total);                //입력
      pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
      pass.setFilterLimits (-2,-1.5);          //적용할 값 (최소, 최대 값)
      pass.filter (*cloud_filtered);

      // ransac - get indices of inliers
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.1);
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);

      // filter inlier points by index
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*pcl_plane);

      extract.setInputCloud (pcl_total);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*pcl_object);

      pcl::toROSMsg(*pcl_plane, pubGroundmsg);
      pubGroundmsg.header.frame_id = "top_velodyne";
      pubGround.publish(pubGroundmsg);

      pcl::toROSMsg(*pcl_object, pubObjectmsg);
      pubObjectmsg.header.frame_id = "top_velodyne";
      pubObject.publish(pubObjectmsg);

      double duration = (double)(time / CLOCKS_PER_SEC);
      cout << "RANSAC done. " << setprecision(5) << duration << "s"<< endl;

    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_detect");

    findGround findG;

    ROS_INFO("\033[1;32m----> [AGV Global Module] : Image Projection / MIDDLE LIDAR\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}

// 해야할 일
// 1. ground 말고 object pcl 도 보내기
//   "/vlp_cloud/object" => /pharos_velodyne_Vcombine_node," //<sensor_msgs::PointCloud2>
//     “vlp_cloud/ground_t" => /pharos_velodyne_Rcombine_node //<sensor_msgs::PointCloud2>
//    이 두 topic을 원래 위 노드에서 합치는데 이번 코드에서는 LRT 구분 없이 한번에 보냄
// 2. 좌표계 odom으로 변환
