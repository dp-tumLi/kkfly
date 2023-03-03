#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"

#include "ros/ros.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class cloud2voxel
{
public:
  ros::NodeHandle nh;
  cloud2voxel() : nh("~")
  {
    
    subscriber_points_ = nh.subscribe("/points", 5, &cloud2voxel::point_cb, this);
    subscriber_tf_ = nh.subscribe("/tf", 10, &cloud2voxel::tf_cb, this);
    // subscriber_test_ = nh.subscribe("/tf", 10, &cloud2voxel::quaternionCallback, this);
    publisher_points_ = nh.advertise<sensor_msgs::PointCloud2>("/points/filtered", 1);
    
    // ros::spin();
  }
  ~cloud2voxel(){};

private:
  ros::Subscriber subscriber_points_;
  ros::Publisher publisher_points_;
  ros::Subscriber subscriber_tf_;//new
  // ros::Subscriber subscriber_test_;
public:
  Eigen::Matrix4f transformation_matrix;


void tf_cb(const tf2_msgs::TFMessage& tf_msg)
{
  transformation_matrix = getTransformationMatrix(tf_msg);
  // ROS_INFO_STREAM("Hello sb");

}

// void quaternionCallback(const tf2_msgs::TFMessage& msg)
// {
//   const auto& transform = msg.transforms[0];
//   tf2::Quaternion quat;
//   tf2::fromMsg(transform.transform.rotation, quat);
//   ROS_INFO("x: %f y: %f z: %f w: %f", quat.getX(),quat.getY(),quat.getZ(),quat.getW());
// }

//new
Eigen::Matrix4f getTransformationMatrix(const tf2_msgs::TFMessage& tf_msg)
{
    // initialize transformation matrix to identity matrix
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    if (tf_msg.transforms.size() > 0) {
        // get the first transform in the message
        const auto& transform = tf_msg.transforms[0];
        // geometry_msgs::TransformStamped transform = tf_msg.transforms[0];


        // convert quaternion to rotation matrix
        tf2::Quaternion quat;
        tf2::fromMsg(transform.transform.rotation, quat);
        tf2::Matrix3x3 rotation_matrix(quat);

        // const auto& temp_matrix = rotation_matrix.transpose();
        // set rotation part of transformation matrix

        transformation_matrix << rotation_matrix[0][0],rotation_matrix[1][0],rotation_matrix[2][0],0,
        rotation_matrix[0][1],rotation_matrix[1][1],rotation_matrix[2][1],0,
        rotation_matrix[0][2],rotation_matrix[1][2],rotation_matrix[2][2],0,
        0,0,0,1;

        // transformation_matrix<<temp_matrix[0][0],temp_matrix[0][1],temp_matrix[0][2],0,
        // temp_matrix[1][0],temp_matrix[1][1],temp_matrix[1][2],0,
        // temp_matrix[2][0],temp_matrix[2][1],temp_matrix[2][2],0,
        // 0,0,0,1;

        transformation_matrix <<1,0,0,0,
                                0,1,0,0,
                                0,0,1,0,
                                0,0,0,1;
        // ROS_INFO("x: %f y: %f z: %f w: %f", quat.getX(),quat.getY(),quat.getZ(),quat.getW());
        // std::cout<<quat.getX()<<quat.getY()<<quat.getZ()<<quat.getW()<<std::endl;
        // // set translation part of transformation matrix
        // transformation_matrix(0,3) = transform.transform.translation.x;
        // transformation_matrix(1,3) = transform.transform.translation.y;
        // transformation_matrix(2,3) = transform.transform.translation.z;
    }

    return transformation_matrix;
}

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
  {
    // Convert from sensor_msgs to pcl::PointCloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*in_cloud_ptr, *cloud);
    // pcl::transformPointCloud(*cloud, *cloud, transformation_matrix);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    pcl::transformPointCloud(*current_pc_ptr, *current_pc_ptr, transformation_matrix);
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);

    // Rotating the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    double rotx = 0.0;
    double roty = M_PI / 2.0;
    double rotz = -M_PI / 2.0;
    Eigen::Matrix4f rotMatrixX;
    Eigen::Matrix4f rotMatrixY;
    Eigen::Matrix4f rotMatrixZ;

    rotMatrixX << 1.0, 0.0, 0.0, 0.0,
        0.0, cos(rotx), -sin(rotx), 0.0,
        0.0, sin(rotx), cos(rotx), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrixY << cos(roty), 0.0, sin(roty), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin(roty), 0.0, cos(roty), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrixZ << cos(rotz), -sin(rotz), 0.0, 0.0,
        sin(rotz), cos(rotz), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
    //new
    
    sensor_msgs::PointCloud2 pub_pc;
    pcl::transformPointCloud(*filtered_pc_ptr, *transformed_cloud_ptr, rotMatrixX * rotMatrixY * rotMatrixZ );
    pcl::toROSMsg(*transformed_cloud_ptr, pub_pc);
    
    pub_pc.header = in_cloud_ptr->header;

    publisher_points_.publish(pub_pc);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cloud_voxel");
  cloud2voxel node;
  ros::spin();
}
