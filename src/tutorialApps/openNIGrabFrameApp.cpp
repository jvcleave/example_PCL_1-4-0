#include "openNIGrabFrameApp.h"

bool no_frame = true;
bool didWriteFile = false;
string dataPath ="";
void openNIGrabFrameApp::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	
	
	if (no_frame)
	{
		
        std::string output_dir;
        if (output_dir.empty ())
			output_dir = ".";
		
        std::string filename;
        if (filename.empty ())
        {
			std::stringstream ss;
			ss << output_dir << "/frame_" << boost::posix_time::to_iso_string(boost::posix_time::microsec_clock::local_time()) << ".pcd";
			filename = ss.str ();
        }
		dataPath = ofToDataPath(filename, true);
		//filename = dataPath+"/"+filename;
		pcl::io::savePCDFileASCII(dataPath, *cloud);
        std::cerr << "Data saved to " << dataPath << std::endl;
		didWriteFile = true;
        no_frame = false;
	}
}

//--------------------------------------------------------------
void openNIGrabFrameApp::setup(){
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	// make callback function from member function
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
	boost::bind (&openNIGrabFrameApp::cloud_cb_, this, _1);
	
	// connect callback function for desired signal. In this case its a point cloud with color values
	boost::signals2::connection c = interface->registerCallback (f);
	
	// start receiving point clouds
	interface->start ();
	
	// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
	while (no_frame)
        boost::this_thread::sleep (boost::posix_time::seconds (1));
	
	// stop the grabber
	interface->stop ();
}

//--------------------------------------------------------------
void openNIGrabFrameApp::update(){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::draw(){
	ofSetColor(255, 0, 0);
	if (didWriteFile)
	{
		ofDrawBitmapString("JUST WROTE A LARGE FILE TO \n" + dataPath, 20, 20);
	}
}

//--------------------------------------------------------------
void openNIGrabFrameApp::keyPressed(int key){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::keyReleased(int key){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void openNIGrabFrameApp::dragEvent(ofDragInfo dragInfo){ 

}