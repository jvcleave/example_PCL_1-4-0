#pragma once

#include "ofMain.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/octree/octree.h>

class pointCloudCompressionExample : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
	void
	cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		//we don't have pcl::visualization :/
		//but you can watch the console!
		
		//if (!viewer.wasStopped ())
		//{
			// stringstream to store compressed point cloud
			std::stringstream compressedData;
			// output pointcloud
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
			
			// compress point cloud
			PointCloudEncoder->encodePointCloud (cloud, compressedData);
			
			// decompress point cloud
			PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
			
			
			// show decompressed point cloud
			//viewer.showCloud (cloudOut);
		//}
	}
		
};


//port of http://pointclouds.org/documentation/tutorials/compression.php#octree-compression