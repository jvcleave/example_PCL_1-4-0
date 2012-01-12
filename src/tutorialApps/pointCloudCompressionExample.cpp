#include "pointCloudCompressionExample.h"


//--------------------------------------------------------------
void pointCloudCompressionExample::setup(){
	bool showStatistics = true;
	
    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	
    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, showStatistics);
    PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
	
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber ();
	
    // make callback function from member function
    boost::function<void
    (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&pointCloudCompressionExample::cloud_cb_, this, _1);
	
    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);
	
    // start receiving point clouds
    interface->start ();
	
}

//--------------------------------------------------------------
void pointCloudCompressionExample::update(){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::draw(){
	
	
}

//--------------------------------------------------------------
void pointCloudCompressionExample::keyPressed(int key){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::keyReleased(int key){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void pointCloudCompressionExample::dragEvent(ofDragInfo dragInfo){ 

}