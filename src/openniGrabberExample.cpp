#include "openniGrabberExample.h"



//--------------------------------------------------------------
void openniGrabberExample::setup(){
	
	// create a new grabber for OpenNI devices
	interface = new pcl::OpenNIGrabber();
	
	// make callback function from member function
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
	boost::bind (&openniGrabberExample::cloud_cb_, this, _1);
	
	// connect callback function for desired signal. In this case its a point cloud with color values
	boost::signals2::connection c = interface->registerCallback (f);
	
	// start receiving point clouds
	interface->start ();
}



//--------------------------------------------------------------
void openniGrabberExample::update(){

}

//--------------------------------------------------------------
void openniGrabberExample::draw(){

}

//--------------------------------------------------------------
void openniGrabberExample::keyPressed(int key){

}

//--------------------------------------------------------------
void openniGrabberExample::keyReleased(int key){

}

//--------------------------------------------------------------
void openniGrabberExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void openniGrabberExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void openniGrabberExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void openniGrabberExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void openniGrabberExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void openniGrabberExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void openniGrabberExample::dragEvent(ofDragInfo dragInfo){ 

}