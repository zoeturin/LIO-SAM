#include <cstdio>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>


// Services
#include "laser_assembler/AssembleScans2.h"

// Messages
#include "sensor_msgs/PointCloud.h"


/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Subscribe to the multisense filtered laser scan
    sub_ = n_.subscribe("laser_scan", 100, &PeriodicSnapshotter::scanCallback, this);

    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("periodic_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans2>("assemble_scans2");

    // Start the timer that will trigger the processing loop (timerCallback)
    // timer_ = n_.createTimer(ros::Duration(0.5), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    // first_time_ = true;
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) //????????
  {
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_) {
      first_time_ = false;
      start_time_ = scan -> header.stamp;
      return;
    }

    if (scan -> angle_min > prev_angle_) {
        prev_angle_ = scan -> angle_min;
        return;
    }
    prev_angle_ = scan -> angle_min;

    // Populate our service request based on our timer callback times
    AssembleScans2 srv;
    srv.request.begin = start_time_;
    srv.request.end   = scan -> header.stamp;

    // Make the service call
    if (client_.call(srv)){
      ROS_INFO("Published Cloud with %u points from %f - %f", (uint32_t)(srv.response.cloud.width), start_time_.toSec(), scan -> header.stamp.toSec() ) ;
      pub_.publish(srv.response.cloud);
    }
    else {
      ROS_ERROR("Error making service call\n") ;
    }
    start_time_ = scan -> header.stamp;
  }

//   void timerCallback(const ros::TimerEvent& e)
//   {

//     // We don't want to build a cloud the first callback, since we we
//     //   don't have a start and end time yet
//     if (first_time_)
//     {
//       first_time_ = false;
//       return;
//     }

//     // Populate our service request based on our timer callback times
//     AssembleScans2 srv;

//     srv.request.begin = e.current_real - ros::Duration(2.0);
//     srv.request.end   = e.current_real;


//     // Make the service call
//     if (client_.call(srv))
//     {
//       ROS_INFO("Published Cloud with %u points from %f - %f", (uint32_t)(srv.response.cloud.width), e
// .last_real.toSec(), e.current_real.toSec() ) ;
//       pub_.publish(srv.response.cloud);
//     }
//     else
//     {
//       ROS_ERROR("Error making service call\n") ;
//     }
//   }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
//   ros::Timer timer_;
  bool first_time_;
  ros::Time start_time_;
  _Float32 prev_angle_; 
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
