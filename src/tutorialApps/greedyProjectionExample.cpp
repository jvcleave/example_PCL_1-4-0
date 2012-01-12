#include "greedyProjectionExample.h"

//--------------------------------------------------------------
void greedyProjectionExample::setup(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 cloud_blob;
	string filePath = ofToDataPath("bun0.pcd", true);
	pcl::io::loadPCDFile (filePath, cloud_blob);
	pcl::fromROSMsg (cloud_blob, *cloud);
	//* the data should be available in cloud
	
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures
	
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);
	
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	
	
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);
	
	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);
	
	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	
	
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	
	cout << "parts size: " << parts.size() << endl;
	cout << "states size: " << states.size() << endl;
	

	
}

//--------------------------------------------------------------
void greedyProjectionExample::update(){

}

//--------------------------------------------------------------
void greedyProjectionExample::draw(){
	ofDrawBitmapString("HIT SPACEBAR TO SAVE VTK", 20, 20);
}

//--------------------------------------------------------------
void greedyProjectionExample::keyPressed(int key){
	if (key == ' ') {
		string outputFilePath = ofToDataPath("", true);
		outputFilePath += ofGetTimestampString()+"_mesh.vtk";
		pcl::io::saveVTKFile (outputFilePath, triangles);
	}
}

//--------------------------------------------------------------
void greedyProjectionExample::keyReleased(int key){

}

//--------------------------------------------------------------
void greedyProjectionExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void greedyProjectionExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void greedyProjectionExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void greedyProjectionExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void greedyProjectionExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void greedyProjectionExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void greedyProjectionExample::dragEvent(ofDragInfo dragInfo){ 

}