#include <ros/ros.h>
#include <point_types.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class LaserScan2PointCloud {
     public:
        LaserScan2PointCloud();
        void scanCallback(pcl::PointCloud<VelodynePointXYZIRT>::Ptr& cloud, sensor_msgs::PointCloud2::Ptr& cloud_temp, const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

LaserScan2PointCloud::LaserScan2PointCloud(){
        // scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &LaserScan2PointCloud::scanCallback, this);
        // point_cloud_publisher_ = node_.advertise<pcl::PointCloud<VelodynePointXYZIRT>> ("/cloud", 100, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserScan2PointCloud::scanCallback(pcl::PointCloud<VelodynePointXYZIRT>::Ptr& cloud, sensor_msgs::PointCloud2::Ptr& cloud_temp, const sensor_msgs::LaserScan::ConstPtr& scan){
//UNSURE: if 
//     projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    projector_.projectLaser(*scan, *cloud_temp);   
//     for (size_t i = 0; i < sizeof(scan->ranges); i++){
    // for (size_t i = 0; i < sizeof(cloud_temp->data); i++){
    //     // auto &src = scan->ranges[i];
    //     auto &src = cloud_temp->data[i];
    //     auto &dst = cloud->points[i];
    //     dst.x = src.x;
    //     dst.y = src.y;
    //     dst.z = src.z;
    //     dst.intensity = src.intensity;
    //     dst.ring = src.ring;
    //     dst.time = src.t * 1e-9f; }
    // // TODO: only publish if complete cloud
    // point_cloud_publisher_.publish(*cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserScan2PointCloud_node");

    LaserScan2PointCloud laserScan2PointCloud;

    ros::spin();

    return 0;
}