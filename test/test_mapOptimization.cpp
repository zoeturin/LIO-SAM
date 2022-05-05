#include "utility.h"
#include <assert.h> 
#include <pcl/point_cloud.h>


PointCloud<PointType>::Ptr cloud1;
PointCloud<PointType>::Ptr cloud2;
float tfvec[6];
mapOptimization opt;


// -------------------- 3 point test ---------------------- //
// test nearest neighbor matches in 3D space using 3 points with rotation and translation
int
test_featureOptimization_simple()
{
    // typedef pcl::PointXYZI PointType;
    // Test 1: simple point cloud
    // initialization
    float x = .1;
    float th = .05; // radians

    cloud1->points = {PointType(0., 0., 0.), 
                      PointType(0., 1., 0.), 
                      PointType(0., 2., 0.)}; // map
                      
    cloud2->points = {PointType(0., x, 0.), 
                      PointType(x + sin(th), cos(th), 0), 
                      PointType(x + 2*sin(th), 2*cos(th), 0)}; // new scan

    pcl::KdTreeFLANN<PointType>::Ptr kdtree1;
    kdtree1 -> setInputCloud(cloud1);
    float tf[6] = {0, 0, 0, 0, 0, 0}; 

    // set mapOptimization members required for featureOptimization()
    opt.kdtreeFeatureFromMap = kdtree1;
    opt.laserCloudFeatureLastDS = cloud2;
    opt.laserCloudFeatureLastDSNum = cloud2 -> size();
    opt.transformTobeMapped = tf;

    opt.featureOptimization();

    for (int i=0; i++; i=3)
    {   
        assert ( opt.featurePointNewVec[i] == cloud2 -> points[i]);
        assert ( opt.featurePointMapVec[i] == cloud1 -> points[i]);
        assert ( opt.rnDistsVec[i] == euclideanDistance(cloud1->points[i], cloud2->points[i]) );
    }
    assert ( opt.featurePointNewMean == cloud2 -> points[0] + cloud2 -> points[1] + cloud2 -> points[2] );
    assert ( opt.featurePointMapMean == cloud1 -> points[0] + cloud1 -> points[1] + cloud1 -> points[2] );

    return 0;
}

int test_optimization()
{
    opt.optimization();
    std::cout << opt.transformTobeMapped;
    return 0;
}

int simple_test()
{
    test_featureOptimization_simple();
    test_optimization();
}

// -------------- ICP test ------------------ //
// test nearest neighbor matches in 3D space using point cloud with applied rotation and translation

int testFeatureOptimization(){
    pcl::KdTreeFLANN<PointType>::Ptr kdtree1;
    kdtree1 -> setInputCloud(cloud1);

    // set mapOptimization members required for featureOptimization()
    opt.kdtreeFeatureFromMap = kdtree1;
    opt.laserCloudFeatureLastDS = cloud2;
    opt.laserCloudFeatureLastDSNum = cloud2 -> size();
    opt.transformTobeMapped = tfvec;

    opt.featureOptimization();
}

int cloud_test(string filename, Eigen::Vector3f tr, Eigen::Vector3f rot)
{
    //NEXT fix include errors
    if (pcl::loadPCDFile<PointXYZ> (filename, *cloud1) < 0)
    {
        std::cerr << "Failed to read test file." << std::endl;
        return (-1);
    }
    Eigen::Transform<float, 3, Eigen::Affine> tf =
        pcl::getTransformation(tr[0], tr[1], tr[2], rot[0], rot[1], rot[2]);
    pcl::transformPointCloud(*cloud1, *cloud2, tf);
    tfvec[0] = tr[0]; tfvec[1] = tr[1]; tfvec[2] = tr[2];
    tfvec[3] = rot[0]; tfvec[4] = rot[1]; tfvec[5] = rot[2];
    
    testFeatureOptimization();
    opt.optimization();
    assert( opt.transformTobeMapped == tf ); //?

    // IndicesPtr indices (new Indices);
    // indices->resize (cloud->size ());

    // for (std::size_t i = 0; i < indices->size (); ++i)
    //     (*indices)[i] = static_cast<int> (i);

}

int
main (int argc, char** argv)
{
    return test_featureOptimization_simple();
}
