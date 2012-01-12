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
	cout << cloud->size() << endl;
	//mutex.lock();
	/**cloud = *_cloud;
	 passthrough.setFilterLimits(minZ.getValue(),maxZ.getValue());
	 passthrough.setInputCloud(cloud);
	 passthrough.filter(*cloud);
	 isNewFrame = true;*/
	//mutex.unlock();
	const size_t num_point = cloud->points.size();
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	
	for (int i = 0; i < num_point; i++)
	{
		pcl::PointXYZRGB p = cloud->points[i];
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
		//ofFloatColor color(p.r, p.g, p.b);
		//mesh.setColor(i, color);
	}
}
//--------------------------------------------------------------
void cameraCloudApp::update(){

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