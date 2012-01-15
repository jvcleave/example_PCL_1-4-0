#include "cameraCloudApp.h"

//--------------------------------------------------------------
void cameraCloudApp::setup(){
	interface = new pcl::OpenNIGrabber();
	
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =	boost::bind (&cameraCloudApp::onCloudData, this, _1);
	interface->registerCallback (f);
	
	interface->start ();
}
void cameraCloudApp::onCloudData (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	const size_t num_point = cloud->points.size();
	if (mesh.getNumVertices() != num_point) 
	{
		mesh.getVertices().resize(num_point);
		mesh.getColors().resize(num_point);
	}	
	float scale = 50.0;
	for (int i = 0; i < num_point; i++)
	{
		
		pcl::PointXYZRGB p = cloud->points[i];
		ofColor pointColor(p.r, p.g, p.b);
		mesh.setVertex(i, ofVec3f(p.x*scale, -p.y*scale, p.z*scale));
		mesh.setColor(i, pointColor);
	}
}
//--------------------------------------------------------------
void cameraCloudApp::update(){
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void cameraCloudApp::draw(){
	camera.begin();
		mesh.drawVertices();
	camera.end();
}

//--------------------------------------------------------------
void cameraCloudApp::keyPressed(int key){

}

//--------------------------------------------------------------
void cameraCloudApp::keyReleased(int key){

}

//--------------------------------------------------------------
void cameraCloudApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void cameraCloudApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void cameraCloudApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void cameraCloudApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void cameraCloudApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void cameraCloudApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void cameraCloudApp::dragEvent(ofDragInfo dragInfo){ 

}
void cameraCloudApp::exit()
{
	interface->stop();
}