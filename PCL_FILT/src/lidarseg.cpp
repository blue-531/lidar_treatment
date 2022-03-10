#include "utility.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class lidar_seg {
private:
public:
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloudInfo;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fusedCloud;

    ros::Publisher pubCombined;
    ros::Publisher pubGround;
    ros::Publisher pubObject;
    sensor_msgs::PointCloud2 pubFusedsensor;

    lidar_seg(){
      // Time synchronization between three lidars
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
      message_filters::Subscriber<sensor_msgs::PointCloud2> *sub1_, *sub2_, *sub3_;
      message_filters::Synchronizer<SyncPolicy>* sync_;
      sub1_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "transformedTop", 1);
      sub2_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "transformedLeft", 1);
      sub3_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "transformedRight", 1);
      sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub1_, *sub2_, *sub3_);
      sync_->registerCallback(boost::bind(&lidar_seg::triple_handler, this, _1, _2, _3));

      pubGround = nh.advertise<sensor_msgs::PointCloud2> ("ground_lidar", 10);
      pubCombined = nh.advertise<sensor_msgs::PointCloud2> ("total_lidar", 10);
      pubObject = nh.advertise<sensor_msgs::PointCloud2> ("object_point", 10);
    }



    void detect_ground_points(const sensor_msgs::PointCloud2ConstPtr& total_msg) {
      //clock_t time = clock();

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_total (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_plane (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_process_1 (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_process_2 (new pcl::PointCloud<pcl::PointXYZI> ());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_object_process_3 (new pcl::PointCloud<pcl::PointXYZI> ());

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      pcl::ExtractIndices<pcl::PointXYZI> extract;

      // filter by z axis only near floorr
      sensor_msgs::PointCloud2 pubGroundmsg;
      sensor_msgs::PointCloud2 pubObjectmsg;

      pcl::fromROSMsg(*total_msg, *pcl_total);
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud (pcl_total);                //입력
      pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
      pass.setFilterLimits (-2,-1.3);          //적용할 값 (최소, 최대 값)
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

      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*pcl_object_process_1);

      pass.setInputCloud (pcl_total);                //입력
      pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
      pass.setFilterLimitsNegative (true);
      pass.setFilterLimits (-3,-1.3);          //적용할 값 (최소, 최대 값)
      pass.filter (*pcl_object_process_2);
      *pcl_object_process_2 += *pcl_object_process_1;

      pass.setInputCloud (pcl_object_process_2);                //입력
      pass.setFilterFieldName ("z");             //적용할 좌표 축 (eg. Z축)
      pass.setFilterLimitsNegative (true);
      pass.setFilterLimits (-3,-1.6);          //적용할 값 (최소, 최대 값)
      pass.filter (*pcl_object_process_3);

      // 차량 자체 lidar 데이터 remove
      pcl::CropBox<pcl::PointXYZI> boxFilter;
      float x_min = -1.5, y_min = -1, z_min = -1.5;
      float x_max = +1.5, y_max = +1, z_max = +0.5;
      boxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
      boxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
      boxFilter.setNegative(true);
      boxFilter.setInputCloud(pcl_object_process_3);
      boxFilter.filter(*pcl_object);




      pcl::toROSMsg(*pcl_object, pubObjectmsg);
      pubObjectmsg.header.frame_id = "top_velodyne";
      pubObject.publish(pubObjectmsg);

      pcl::toROSMsg(*pcl_plane, pubGroundmsg);
      pubGroundmsg.header.frame_id = "top_velodyne";
      pubGround.publish(pubGroundmsg);

      //double duration = (double)(time / CLOCKS_PER_SEC);
      //cout << "RANSAC done. " << setprecision(5) << duration << "s"<< endl;

    }

    void triple_handler(const sensor_msgs::PointCloud2ConstPtr& TopPC, const sensor_msgs::PointCloud2ConstPtr& LeftPC, const sensor_msgs::PointCloud2ConstPtr& RightPC){
        pcl::PointCloud<pcl::PointXYZI>::Ptr fusedCloud (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclT (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclL (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pclR (new pcl::PointCloud<pcl::PointXYZI> ());
        pcl::fromROSMsg(*TopPC, *pclT);
        pcl::fromROSMsg(*LeftPC, *pclL);
        pcl::fromROSMsg(*RightPC, *pclR);
        fusedCloud->clear();
        *fusedCloud += *pclT;
        *fusedCloud += *pclL;
        *fusedCloud += *pclR;

        pcl::toROSMsg(*fusedCloud, pubFusedsensor);
        pubFusedsensor.header.frame_id = "top_velodyne";
        pubCombined.publish(pubFusedsensor);

        const sensor_msgs::PointCloud2ConstPtr& Fusedcloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>(pubFusedsensor);
        detect_ground_points(Fusedcloud_ptr);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidarseg_node");

    lidar_seg lidarseg;

    ROS_INFO("\033[1;32m----> [AGV Global Module] : Image Projection / MIDDLE LIDAR\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
