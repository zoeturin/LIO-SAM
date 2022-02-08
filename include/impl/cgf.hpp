#pragma once
#include <include/cgf.h>

// CGF member function definitions for feature computation:
template <typename PointInT, typename PointOutT> void
pcl::CGFEstimation<PointInT, PointOutT>::computeSphericalHistogram ()
{
    // NN range search
    // iterate through points and increment bins
}

template <typename PointInT, typename PointOutT> void
pcl::CGFEstimation<PointInT, PointOutT>::computeCGFSignature ()
{

}

template <typename PointInT, typename PointOutT> void
pcl::CGFEstimation<PointInT, PointOutT>::computeCGFSignatures ()
{
    // iterate through input cloud
    computeCGFSignature();
}

template <typename PointInT, typename PointOutT> void
pcl::CGFEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
    computeCGFSignatures();

}



