#include "resamplingExample.h"

//--------------------------------------------------------------
void resamplingExample::setup(){
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// Load bun0.pcd -- should be available with the PCL archive in test 
	string filePath = ofToDataPath("bun0.pcd", true);
	pcl::io::loadPCDFile (filePath, *cloud);
	
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	
	// Output has the same type as the input one, it will be only smoothed
	pcl::PointCloud<pcl::PointXYZ> mls_points;
	
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;
	
	// Optionally, a pointer to a cloud can be provided, to be set by MLS
	pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	mls.setOutputNormals (mls_normals);
	
	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);
	
	// Reconstruct
	mls.reconstruct (mls_points);
	
	// Concatenate fields for saving

	
	pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);
	
	
	
	
}

//--------------------------------------------------------------
void resamplingExample::update(){

}

//--------------------------------------------------------------
void resamplingExample::draw(){
	ofDrawBitmapString("HIT SPACEBAR TO SAVE PCD", 20, 20);
}

//--------------------------------------------------------------
void resamplingExample::keyPressed(int key){
	if (key == ' ') {
		string outputFilePath = ofToDataPath("", true);
		outputFilePath += ofGetTimestampString()+"_resamplingExample_bun0-mls.pcd";
		// Save output
		pcl::io::savePCDFile (outputFilePath, mls_cloud);
	}
}

//--------------------------------------------------------------
void resamplingExample::keyReleased(int key){

}

//--------------------------------------------------------------
void resamplingExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void resamplingExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void resamplingExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void resamplingExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void resamplingExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void resamplingExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void resamplingExample::dragEvent(ofDragInfo dragInfo){ 

}