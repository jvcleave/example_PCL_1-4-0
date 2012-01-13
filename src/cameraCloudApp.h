#pragma once

#include "ofMain.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/keypoints/sift_keypoint.h>

class cameraCloudApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
	void exit();
		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void onCloudData (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
		pcl::Grabber* interface;
		ofMesh mesh;
		ofEasyCam camera;
};
