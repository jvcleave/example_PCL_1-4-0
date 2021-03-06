/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: search.h 3536 2011-12-14 22:24:39Z rusu $
 *
 */

#ifndef PCL_SEARCH_SEARCH_H_
#define PCL_SEARCH_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>

namespace pcl
{
  namespace search
  {
    /** \brief Generic search class. All search wrappers must inherit from this. 
      * 
      * Each search method must implement 2 different types of search:
      *   - \b nearestKSearch - search for K-nearest neighbors. 
      *   - \b radiusSearch - search for all nearest neighbors in a sphere of a given radius
      *
      * The input to each search method can be given in 3 different ways:
      *   - as a query point
      *   - as a (cloud, index) pair
      *   - as an index
      *
      * For the latter option, it is assumed that the user specified the input
      * via a \ref setInputCloud () method first.
      *
      * \note In case of an error, all methods are supposed to return 0 as the number of neighbors found.
      *
      * \note libpcl_search deals with three-dimensional search problems. For higher
      * level dimensional search, please refer to the libpcl_kdtree module.
      *
      * \ingroup search
      */
    template<typename PointT>
    class Search
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<pcl::search::Search<PointT> > Ptr;
        typedef boost::shared_ptr<const pcl::search::Search<PointT> > ConstPtr;

        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        /** Constructor. */
        Search ()
        {
        }

        /** Destructor. */
        virtual
        ~Search ()
        {
        }

        /** \brief Pass the input dataset that the search will be performed on.
          * \param[in] cloud a const pointer to the PointCloud data
          * \param[in] indices the point indices subset that is to be used from the cloud
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud, const IndicesConstPtr& indices) {}

        /** \brief Pass the input dataset that the search will be performed on.
          * \param[in] cloud a const pointer to the PointCloud data
          */
        virtual void
        setInputCloud (const PointCloudConstPtr& cloud) = 0;

        /** \brief Get a pointer to the input point cloud dataset. */
        virtual PointCloudConstPtr
        getInputCloud () = 0;

        /** \brief Get a pointer to the vector of indices used. */
        virtual IndicesConstPtr const
        getIndices () { return (IndicesConstPtr ()); }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        virtual int
        nearestKSearch (const PointT &point, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances) = 0;

        /** \brief Search for the k-nearest neighbors for the given query point. Use this method if the query points are of a different type than the points in the data set (e.g. PointXYZRGB instead of PointXYZ).
          * \param[in] point the given query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        template <typename PointTDiff> int
        nearestKSearchT (const PointTDiff &point, int k, std::vector<int> &k_indices,
                         std::vector<float> &k_sqr_distances)
        {
          PointT p;
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
          pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (
                point, p));
          return (nearestKSearch (p, k, k_indices, k_sqr_distances));
        }

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
          * a priori!)
          * \return number of neighbors found
          */
        virtual int
        nearestKSearch (const PointCloud& cloud, int index, int k, std::vector<int>& k_indices,
                        std::vector<float>& k_sqr_distances) = 0;

        /** \brief Search for the k-nearest neighbors for the given query point (zero-copy).
         * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
         *        if indices were given in setInputCloud, index will be the position in the indices vector
         * \param[in] k the number of neighbors to search for
         * \param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
         * \param[out] k_sqr_distances the resultant squared distances to the neighboring points (must be resized to \a k
         * a priori!)
         * \return number of neighbors found
         */
        virtual int
        nearestKSearch (int index, int k, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances) = 0;

        /** \brief Search for the k-nearest neighbors for the given query point.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          */
        virtual void
        nearestKSearch (const PointCloud& cloud, const std::vector<int>& indices, int k, std::vector< std::vector<int> >& k_indices,
                        std::vector< std::vector<float> >& k_sqr_distances)
        {
          if (indices.empty ())
          {
            k_indices.resize (cloud.size ());
            k_sqr_distances.resize (cloud.size ());
            for (size_t i = 0; i < cloud.size (); i++)
              nearestKSearch (cloud,(int) i,k,k_indices[i],k_sqr_distances[i]);
          }
          else
          {
            k_indices.resize (indices.size ());
            k_sqr_distances.resize (indices.size ());
            for (size_t i = 0; i < indices.size (); i++)
              nearestKSearch (cloud,indices[i],k,k_indices[i],k_sqr_distances[i]);
          }
        }

        /** \brief Search for the k-nearest neighbors for the given query point. Use this method if the query points are of a different type than the points in the data set (e.g. PointXYZRGB instead of PointXYZ).
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] k the number of neighbors to search for
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \note This method copies the input point cloud of type PointTDiff to a temporary cloud of type PointT and performs the batch search on the new cloud. You should prefer the single-point search if you don't use a search algorithm that accelerates batch NN search.
          */
        template <typename PointTDiff> void
        nearestKSearchT (const pcl::PointCloud<PointTDiff> &cloud, const std::vector<int>& indices, int k, std::vector< std::vector<int> > &k_indices,
                         std::vector< std::vector<float> > &k_sqr_distances)
        {
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;

          pcl::PointCloud<PointT> pc;
          if (indices.empty ())
          {
            pc.resize (cloud.size());
            for (size_t i = 0; i < cloud.size(); i++)
            {
              pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (
                                              cloud[i], pc[i]));
            }
            nearestKSearch (pc,std::vector<int>(),k,k_indices,k_sqr_distances);
          }
          else
          {
            pc.resize (indices.size());
            for (size_t i = 0; i < indices.size(); i++)
            {
              pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (
                                              cloud[indices[i]], pc[i]));
            }
            nearestKSearch (pc,std::vector<int>(),k,k_indices,k_sqr_distances);
          }
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (const PointT& point, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_sqr_distances, int max_nn = -1) const = 0;

        /** \brief Search for all the nearest neighbors of the query point in a given radius. Use this method if the query points are of a different type than the points in the data set (e.g. PointXYZRGB instead of PointXYZ).
          * \param[in] point the given query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        template <typename PointTDiff> int
        radiusSearchT (const PointTDiff& point, double radius, std::vector<int>& k_indices,
                       std::vector<float>& k_sqr_distances, int max_nn = -1)
        {
          PointT p;
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
          pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (
                point, p));
          return (radiusSearch (p, radius, k_indices, k_sqr_distances, max_nn));
        }

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] index the index in \a cloud representing the query point
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (const PointCloud& cloud, int index, double radius, std::vector<int>& k_indices,
                      std::vector<float>& k_sqr_distances, int max_nn = -1) = 0;

        /** \brief search for all the nearest neighbors of the query point in a given radius (zero-copy).
          * \param[in] index the index representing the query point in the dataset given by \a setInputCloud
          *        if indices were given in setInputCloud, index will be the position in the indices vector
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          * \return number of neighbors found in radius
          */
        virtual int
        radiusSearch (int index, double radius, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances,
                      int max_nn = -1) const = 0;

        /** \brief Search for all the nearest neighbors of the query point in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] indices the indices in \a cloud. If indices is empty, neighbors will be searched for all points.
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors to this value
          */
        virtual void
        radiusSearch (const PointCloud& cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> >& k_indices,
                std::vector< std::vector<float> > & k_sqr_distances, int max_nn = -1)
        {
          if (indices.empty ())
          {
            k_indices.resize (cloud.size ());
            k_sqr_distances.resize (cloud.size ());
            for (size_t i = 0; i < cloud.size (); i++)
              radiusSearch (cloud,(int) i,radius,k_indices[i],k_sqr_distances[i], max_nn);
          }
          else
          {
            k_indices.resize (indices.size ());
            k_sqr_distances.resize (indices.size ());
            for (size_t i = 0; i < indices.size (); i++)
              radiusSearch (cloud,indices[i],radius,k_indices[i],k_sqr_distances[i], max_nn);
          }
        }


        /** \brief Search for all the nearest neighbors of the query points in a given radius.
          * \param[in] cloud the point cloud data
          * \param[in] indices a vector of point cloud indices to query for nearest neighbors
          * \param[in] radius the radius of the sphere bounding all of p_q's neighbors
          * \param[out] k_indices the resultant indices of the neighboring points, k_indices[i] corresponds to the neighbors of the query point i
          * \param[out] k_sqr_distances the resultant squared distances to the neighboring points, k_sqr_distances[i] corresponds to the neighbors of the query point i
          * \param[in] max_nn if given, bounds the maximum returned neighbors per query point to this value
          * \note This method copies the input point cloud of type PointTDiff to a temporary cloud of type PointT and performs the batch search on the new cloud. You should prefer the single-point search if you don't use a search algorithm that accelerates batch NN search.
          */
        template <typename PointTDiff> void
        radiusSearchT (const pcl::PointCloud<PointTDiff> &cloud, const std::vector<int>& indices, double radius, std::vector< std::vector<int> > &k_indices,
                         std::vector< std::vector<float> > & k_sqr_distances, int max_nn = -1)
        {
          // Copy all the data fields from the input cloud to the output one
          typedef typename pcl::traits::fieldList<PointT>::type FieldListInT;
          typedef typename pcl::traits::fieldList<PointTDiff>::type FieldListOutT;
          typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;

          pcl::PointCloud<PointT> pc;
          if (indices.empty ())
          {
            pc.resize (cloud.size ());
            for (size_t i = 0; i < cloud.size (); ++i)
              pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (cloud[i], pc[i]));
            radiusSearch (pc, std::vector<int> (), radius, k_indices, k_sqr_distances, max_nn);
          }
          else
          {
            pc.resize (indices.size ());
            for (size_t i = 0; i < indices.size (); ++i)
              pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointTDiff, PointT> (cloud[indices[i]], pc[i]));
            radiusSearch (pc, std::vector<int>(), radius, k_indices, k_sqr_distances, max_nn);
          }
        }
    };
  }
}

#endif  //#ifndef _PCL_SEARCH_GENERIC_SEARCH_H_
