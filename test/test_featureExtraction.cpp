#include "utility.h"
#include <assert.h> 


pcl::PointCloud<PointType>::Ptr cloud;
FeatureExtraction feat;


// -------------------- 6 point test ---------------------- //
// test feature detection and extraction with a set of 6 points

/** inputs to detectFeatures(): 
 * N_SCAN 
 * cloudInfo: startRingIndex, endRingIndex, pointColInd
 * cloudSmoothness: from calculateSmoothness()
 * cloudNeighborPicked: encodes occlusion prior to detectFeatures() execution
 * cloudCurvature
 * edgeThreshold
 * extractedCloud: scan
**/

int test_detection(){

}

int test_extraction(){

}

int simple_test(){
    // set curvature threshold for test?
    int N = 10;
    Eigen::Vector3f line_start(0., 0., 0.);
    Eigen::Vector3f line_end(0., 10., 0.);
    Eigen::Vector3f line_diff;
    line_diff = (line_end.array() - line_start.array()) / N;

    // set to scan not entire cloud?
    for (int i=0; i<N; i++){
        Eigen::Vector3f newpt;
        newpt = line_start.array() + (N * line_diff).array();
        cloud->points.push_back(PointType(newpt[0], newpt[1], newpt[2]));
    }

    feat.N_SCAN = 1;
    lio_sam::cloud_info cloudInfo;
    feat.cloudInfo = cloundInfo;
    feat.cloudInfo.startRingIndex = 0; //?
    feat.cloudInfo.endRingIndex = N; //?
    //feat.cloudInfo.pointColInd = 
    // feat.cloudSmoothness =
    // feat.cloudCurvature =
    // feat.cloudNeighborPicked =
    // feat.featureThreshold = 
    feat.extractedCloud = *cloud;

    
}


// -------------- Cloud test ------------------ //
// test nearest neighbor matches in 3D space using point cloud with applied rotation and translation

int cloud_test(string filename, Eigen::Vector3f tr, Eigen::Vector3f rot)
{
    //NEXT fix include errors
    if (pcl::loadPCDFile<PointXYZ> (filename, *cloud) < 0)
    {
        std::cerr << "Failed to read test file." << std::endl;
        return (-1);
    }

}

int
main (int argc, char** argv)
{
    return 0;
}
