	
 #ifndef PCL_FEATURES_CGF_H_
 #define PCL_FEATURES_CGF_H_

 #include <pcl/features/feature.h>

 namespace pcl
 {
   template<typename PointT>
   class SphericalHistogram : public Feature<PointT>
   {
      public:
        SphericalHistogram(int az_div, int el_div, int rad_div, float model_diam, PointT point, pcl::PointCloud<PointT>::Ptr &cloud):
          az_div_ (az_div),
          el_div_ (el_div),
          rad_div_ (rad_div),
          model_diam_ (model_diam);
          point_ (point)
        {
          generate_hist(point, &cloud)
        }

      private:
        int az_div_;
        int el_div_;
        int rad_div_;
        float model_diam_;

        PointT point_;
        Eigen::VectorXf sph_hist_;

        void 
        generate_hist(PointT point, 
                      pcl::PointCloud<PointT>::Ptr &cloud)
        {
          // nearest neighbors
          // empty cloud for neighbors
          pcl::PointCloud<PointT>::Ptr nns (new pcl::PointCloud<PointT> ());
          // search
          pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<pcl::PointT> ());
          self.setSearchMethod (tree);
          self.setRadiusSearch(//TODO)
          self.compute(*nns)

          // bin points
          
        }
   };

   template<typename PointT>
   class CGF : public Feature<PointT>
   {
      public: 
          CGF() : 
          {
          }

      private:
          
          SphericalHistogram sph_hist_;

   };
 }

 #endif // PCL_FEATURES_CGF_H_
