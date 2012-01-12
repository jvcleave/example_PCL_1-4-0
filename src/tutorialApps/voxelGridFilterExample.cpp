#include "voxelGridFilterExample.h"

//--------------------------------------------------------------
void voxelGridFilterExample::setup(){
	
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ());
	sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
	
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read (ofToDataPath("table_scene_lms400.pcd"), *cloud); // Remember to download the file first!
	
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	<< " data points (" << pcl::getFieldsList (*cloud) << ").";
	
	// Create the filtering object
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);
	
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
	
	pcl::PCDWriter writer;
	writer.write (ofToDataPath("table_scene_lms400_downsampled.pcd"), *cloud_filtered, 
				  Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
}

//--------------------------------------------------------------
void voxelGridFilterExample::update(){

}

//--------------------------------------------------------------
void voxelGridFilterExample::draw(){
	
	
}

//--------------------------------------------------------------
void voxelGridFilterExample::keyPressed(int key){

}

//--------------------------------------------------------------
void voxelGridFilterExample::keyReleased(int key){

}

//--------------------------------------------------------------
void voxelGridFilterExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void voxelGridFilterExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void voxelGridFilterExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void voxelGridFilterExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void voxelGridFilterExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void voxelGridFilterExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void voxelGridFilterExample::dragEvent(ofDragInfo dragInfo){ 

}