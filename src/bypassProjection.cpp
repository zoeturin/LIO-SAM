
#include <cstdio>
#include <ros/ros.h>
#include "utility.h"
#include "point_types.h"

// Messages
#include "sensor_msgs/PointCloud.h"
#include "lio_sam/cloud_info.h"

using PointXYZIRT = VelodynePointXYZIRT;

class BypassProjection : public ParamServer
{

public:
  BypassProjection()
  {
    allocateMemory();
    sub = n.subscribe("assembed_cloud", 100, &BypassProjection::publishClouds, this);
    // Create a publisher for the clouds that we assemble
    pub = n.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_info", 1);
  }

void allocateMemory()
    {
        extractedCloud.reset(new pcl::PointCloud<PointXYZIRT>());
        // tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());

        // fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        extractedCloud->clear();
    }

void publishClouds(const sensor_msgs::PointCloud2::Ptr& cloud)
    {
        // pointCloud2toPointCloud(cloud);
        cloudInfo.header = cloud -> header;
        cloudInfo.cloud_deskewed = *cloud;
        // cloudInfo.cloud_deskewed  = publishCloud(&pub, extractedCloud, cloud -> header.stamp, lidarFrame);
        pub.publish(cloudInfo);
    }

// void pointCloud2toPointCloud(const sensor_msgs::PointCloud2::Ptr& cloud)
//     {
//       if (sensor == SensorType::VELODYNE)
//         {
//             pcl::moveFromROSMsg(*cloud, *extractedCloud);
//         }
//       else if (sensor == SensorType::OUSTER)
//         {
//             // Convert to Velodyne format
//             pcl::moveFromROSMsg(*cloud, *extractedCloud); // PointCloud2 to PointCloud<T>
//             extractedCloud->points.resize(tmpOusterCloudIn->size());
//             extractedCloud->is_dense = tmpOusterCloudIn->is_dense;
//             for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
//             {
//                 auto &src = tmpOusterCloudIn->points[i];
//                 auto &dst = extractedCloud->points[i];
//                 dst.x = src.x;
//                 dst.y = src.y;
//                 dst.z = src.z;
//                 dst.intensity = src.intensity;
//                 dst.ring = src.ring;
//                 dst.time = src.t * 1e-9f;
//             }
//         }
//     }



private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  // pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<PointXYZIRT>::Ptr extractedCloud;
  lio_sam::cloud_info cloudInfo;
} ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bypass_projection");
  ros::NodeHandle n;
  BypassProjection bypass;
  ros::spin();
  return 0;
}
