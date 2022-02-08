	
//  #ifndef PCL_FEATURES_CGF_H_
//  #define PCL_FEATURES_CGF_H_
#pragma once
#include <pcl/features/feature.h>

namespace pcl
{
  // template<typename PointT>
  // class SphericalHistogram : public Feature<PointT>
  // {
  //   public:
  //     SphericalHistogram(int az_div, int el_div, int rad_div, float model_diam, PointT point, pcl::PointCloud<PointT>::Ptr &cloud):
  //       az_div_ (az_div),
  //       el_div_ (el_div),
  //       rad_div_ (rad_div),
  //       model_diam_ (model_diam);
  //       point_ (point)
  //     {
  //       generate_hist(point, &cloud)
  //     }

  //   private:
  //     int az_div_;
  //     int el_div_;
  //     int rad_div_;
  //     float model_diam_;

  //     PointT point_;
  //     Eigen::VectorXf sph_hist_;

  //     void 
  //     generate_hist(PointT point, 
  //                   pcl::PointCloud<PointT>::Ptr &cloud)
  //     {
  //       // nearest neighbors
  //       // empty cloud for neighbors
  //       pcl::PointCloud<PointT>::Ptr nns (new pcl::PointCloud<PointT> ());
  //       // search
  //       pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointT> ());
  //       self.setSearchMethod (tree);
  //       self.setRadiusSearch(//TODO)
  //       self.compute(*nns)
  //       // bin points
        
  //     }
  // };

  template<typename PointInT, typename PointOutT>
  class CGFEstimation : public Feature<PointInT, PointOutT>
  {
    public: 
      using Ptr = shared_ptr<FPFHEstimation<PointInT, PointNT, PointOutT> >;
      using ConstPtr = shared_ptr<const CGFEstimation<PointInT, PointOutT> >;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::search_radius_;
      // using Feature<PointInT, PointOutT>::search_parameter_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;

      using PointCloudOut = typename Feature<PointInT, PointOutT>::PointCloudOut;

      // Empty constructor:
      CGF() : 
      az_div_ (), el_div (), rad_div_ ()//, cloud_diam_ () 
      {
        feature_name_ = "CGFEstimation";
      };

      // Computation member function declarations
      void 
      computeSphericalHistogram(); // TODO: params

      void 
      computeCGFSignature (); // TODO: params

      // Define getters and setters:
      inline void
      getHistogramDivisions (int az_div, int el_div, int rad_div)
      {
        az_div_ = az_div;
        el_div_ = el_div;
        rad_div_ = rad_div;
      }

      inline void
      setHistogramDivisions (int &az_div, int &el_div, int &rad_div)
      {
        az_div = az_div_;
        el_div = el_div_;
        rad_div = rad_div_;
        // TODO: new sph_hist_ of correct size
      }

    protected:
      /** \brief Estimate the set of all CGF (Compact Geometric Feature) signatures for the input cloud
        * 
        */
      void 
      computeCGFSignatures ();

      void 
      computeFeature (PointCloudOut &output) override;

        // for（int i = 0； i < cornerCloud->points.size(); i++）{
        //     // clear current feature
        //     computeCGF(cornerCloud->points[i], currentFeature);
        //     featureCloud->push_back(currentFeature);
        // }

      // Histogram parameters:
      int az_div_, el_div_, rad_div_;
      //int cloud_diam_; //TODO: ?


      SphericalHistogram sph_hist_;

  };
}

//  #endif // PCL_FEATURES_CGF_H_
