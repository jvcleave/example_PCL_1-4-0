#pragma once

#include "ofMain.h"
#include <XnCppWrapper.h>
#include <XnTypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/pcl_common_time.h>


class openniGrabberExample : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		static unsigned count = 0;
		static double last = pcl::getTime ();
		if (++count == 30)
		{
			double now = pcl::getTime ();
			std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
			count = 0;
			last = now;
		}
	}
	
	pcl::Grabber* interface;
		
};
