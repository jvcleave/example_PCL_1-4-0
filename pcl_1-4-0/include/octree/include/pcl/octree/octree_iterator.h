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
 * $Id: octree_iterator.h 3749 2011-12-31 22:58:01Z rusu $
 */

#ifndef OCTREE_ITERATOR_H
#define OCTREE_ITERATOR_H

#include <cstddef>
#include <vector>

#include "octree_nodes.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iterator>

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree iterator class
      * \note This class implements a forward iterator for traversing octrees.
      * \ingroup octree
      * \author Julius Kammerl (julius@kammerl.de)
      */
    template<typename DataT, typename LeafT, typename OctreeT>
    class OctreeNodeIterator: public std::iterator<std::forward_iterator_tag, const OctreeNode, void,
          const OctreeNode*, const OctreeNode&>
    {
      // public typedefs
      typedef typename OctreeT::OctreeBranch OctreeBranch;
      typedef typename OctreeT::OctreeKey OctreeKey;

      public:
        /** \brief Constructor.
          * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
          */
        explicit
        OctreeNodeIterator (const OctreeT& octree_arg);

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeNodeIterator ();

        /** \brief Reset the iterator to the root node of the octree
          */
        inline void
        reset ();

        /** \brief Get octree key for the current iterator octree node
          * \return octree key of current node
          */
        inline const OctreeKey&
        getCurrentOctreeKey () const
        {
          return (currentOctreeKey_);
        }

        /** \brief Get the current depth level of octree
          * \return depth level
          */
        inline unsigned int
        getCurrentOctreeDepth () const
        {
          return (currentOctreeDepth_);
        }

        /** \brief Get the current octree node
          * \return pointer to current octree node
          */
        inline const OctreeNode*
        getCurrentOctreeNode () const
        {
          return (currentNode_);
        }

        /** \brief Generate bounds of the current octree voxel
          * \param[out] min_pt lower bound of voxel
          * \param[out] max_pt upper bound of voxel
          */
        inline void
        getVoxelBounds (Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt) const
        {
          octree_->genVoxelBoundsFromOctreeKey (currentOctreeKey_, currentOctreeDepth_, min_pt, max_pt);
        }

        /** \brief *operator.
          * \return pointer to the current octree node
          */
        inline const OctreeNode*
        operator* () const
        { // return designated object
          return (this->getCurrentOctreeNode ());
        }

        /** \brief Equality operator.
          * \return return true if two OctreeNodeIterator are equal. False otherwise.
          */
        inline bool
        operator== (const OctreeNodeIterator& right_arg) const
        { // test for iterator equality
          return ((octree_ == right_arg.octree_) && (currentNode_ == right_arg.currentNode_));
        }

        /** \brief Inequality operator.
          * \return return true if two OctreeNodeIterator are not equal. False otherwise.
          */
        inline bool
        operator!= (const OctreeNodeIterator& right_arg) const
        { // test for iterator inequality
          return (!(octree_ != right_arg.octree_) || !(currentNode_ != right_arg.currentNode_));
        }

        /** \brief Skip all child voxels of current node and return to parent node.
          */
        void
        skipChildVoxels ();

        /** \brief Preincrement operator.
          * \note recursively step to next octree node
          */
        OctreeNodeIterator&
        operator++ ();

        /** \brief postincrement operator.
          * \note recursively step to next octree node
          */
        inline OctreeNodeIterator
        operator++ (int)
        {
          OctreeNodeIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

      protected:
        /** \brief Reference to octree class. */
        const OctreeT& octree_;

        /** Pointer to current octree node. */
        const OctreeNode* currentNode_;

        /** Child index at current octree node. */
        unsigned char currentChildIdx_;

        /** Depth level in the octree structure. */
        unsigned int currentOctreeDepth_;

        /** Octree key for current octree node. */
        OctreeKey currentOctreeKey_;

        /** Stack structure. */
        std::vector<std::pair<OctreeNode const*, unsigned char> > stack_;
      };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Octree leaf node iterator class
      * \note This class implements a forward iterator for traversing the leaf node an octree data structure.
      * \ingroup octree
      * \author Julius Kammerl (julius@kammerl.de)
      */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename DataT, typename LeafT, typename OctreeT>
    class OctreeLeafNodeIterator : public OctreeNodeIterator<DataT, LeafT, OctreeT>
    {
      public:
        /** \brief Constructor.
          * \param[in] octree_arg Octree to be iterated. Initially the iterator is set to its root node.
          */
        explicit
        OctreeLeafNodeIterator (const OctreeT& octree_arg) :
          OctreeNodeIterator<DataT, LeafT, OctreeT> (octree_arg)
        {
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeLeafNodeIterator ()
        {
        }

        /** \brief Preincrement operator.
          * \note recursively step to next octree leaf node
          */
        inline OctreeLeafNodeIterator&
        operator++ ()
        {
          do
          {
            OctreeNodeIterator<DataT, LeafT, OctreeT>::operator++ ();
          } while ((this->currentNode_) && (this->currentNode_->getNodeType () != LEAF_NODE));

          return (*this);
        }

        /** \brief Postincrement operator.
          * \note recursively step to next octree leaf node
          */
        inline OctreeLeafNodeIterator
        operator++ (int)
        {
          OctreeLeafNodeIterator _Tmp = *this;
          ++*this;
          return (_Tmp);
        }

        /** \brief *operator.
          * \return const pointer to the current octree leaf node
          */
        const LeafT*
        operator* () const
        {
          // return designated object
          const LeafT* ret = NULL;

          if (this->currentNode_ && (this->currentNode_->getNodeType () == LEAF_NODE))
            ret = (const LeafT*)this->currentNode_;
          return (ret);
        }

        /** \brief Method for retrieving a single DataT element from the octree leaf node
          * \param[in] data_arg reference to return pointer of leaf node DataT element.
          */
        virtual void
        getData (const DataT*& data_arg) const
        {
          const DataT* result = 0;

          if (this->currentNode_ && (this->currentNode_->getNodeType () == LEAF_NODE))
          {
            const LeafT* leafNode = (const LeafT*)this->currentNode_;
            leafNode->getData (result);
          }
          data_arg = result;
        }

        /** \brief Method for retrieving a vector of DataT elements from the octree laef node
          * \param[out] dataVector_arg reference to DataT vector that is extended with leaf node DataT elements.
          */
        virtual void
        getData (std::vector<DataT>& dataVector_arg) const
        {
          if (this->currentNode_ && (this->currentNode_->getNodeType () == LEAF_NODE))
          {
            const LeafT* leafNode = (const LeafT*)this->currentNode_;
            leafNode->getData (dataVector_arg);
          }
        }
      };

  }
}

#endif

