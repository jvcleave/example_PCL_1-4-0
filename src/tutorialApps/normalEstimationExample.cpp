#include "normalEstimationExample.h"

//--------------------------------------------------------------
void normalEstimationExample::setup(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 cloud_blob;
	string filePath = ofToDataPath("bun0.pcd", true);
	pcl::io::loadPCDFile (filePath, cloud_blob);
	pcl::fromROSMsg (cloud_blob, *cloud);
	
	
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);
	
	// Compute the features
	ne.compute (*cloud_normals);
	
	cout << "cloud_normals->points.size (): " << cloud_normals->points.size () << endl;
	//To compute a single point normal, use:
	//computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);

}

//--------------------------------------------------------------
void normalEstimationExample::update(){

}

//--------------------------------------------------------------
void normalEstimationExample::draw(){
}

//--------------------------------------------------------------
void normalEstimationExample::keyPressed(int key){
	
}

//--------------------------------------------------------------
void normalEstimationExample::keyReleased(int key){

}

//--------------------------------------------------------------
void normalEstimationExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void normalEstimationExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void normalEstimationExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void normalEstimationExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void normalEstimationExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void normalEstimationExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void normalEstimationExample::dragEvent(ofDragInfo dragInfo){ 

}