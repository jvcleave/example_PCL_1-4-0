#include "statisticalOutlierRemovalExample.h"

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::setup(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZ> (ofToDataPath("table_scene_lms400.pcd"), *cloud);
	
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);
	
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> (ofToDataPath("table_scene_lms400_inliers.pcd"), *cloud_filtered, false);
	
	sor.setNegative (true);
	sor.filter (*cloud_filtered);
	writer.write<pcl::PointXYZ> (ofToDataPath("table_scene_lms400_outliers.pcd"), *cloud_filtered, false);
	
}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::update(){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::draw(){
	
	
}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::keyPressed(int key){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::keyReleased(int key){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void statisticalOutlierRemovalExample::dragEvent(ofDragInfo dragInfo){ 

}