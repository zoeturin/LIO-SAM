#include "utility.h"


pcl::PointCloud<PointType>::Ptr cloud1;
pcl::PointCloud<PointType>::Ptr cloud2;
mapOptimization opt;



int
test_featureOptimization()
{
    // typedef pcl::PointXYZI PointType;
    // Test 1: simple point cloud
    // initialization
    float x = .1;
    float th = .05; // radians

    cloud1->points = {PointType(0., 0., 0.), 
                      PointType(0., 1., 0.), 
                      PointType(0., 2., 0.)};
                      
    cloud2->points = {PointType(0., x, 0.), 
                      PointType(x + sin(th), cos(th), 0), 
                      PointType(x + 2*sin(th), 2*cos(th), 0)};

    pcl::KdTreeFLANN<PointType>::Ptr kdtree1;
    kdtree1 -> setInputCloud(cloud1);
    float tf[6] = {0, 0, 0, 0, 0, 0}; 

    // set mapOptimization members required for featureOptimization()
    opt.kdtreeFeatureFromMap = kdtree1;
    opt.laserCloudFeatureLastDS = cloud2;
    opt.laserCloudFeatureLastDSNum = cloud2 -> size();
    opt.transformTobeMapped = tf;
    
    return 0;
}

int test_optimization()
{
    opt.optimization();
    std::cout << tf;
}

int
main (int argc, char** argv)
{
    return test_featureOptimization();
}