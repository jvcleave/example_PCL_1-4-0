#include "kdtreeSearchExample.h"

//--------------------------------------------------------------
void kdtreeSearchExample::setup(){
	srand (time (NULL));
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0);
		cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0);
		cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0);
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	
	kdtree.setInputCloud (cloud);
	
	pcl::PointXYZ searchPoint;
	
	searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0);
	searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0);
	searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0);
	
	// K nearest neighbor search
	
	int K = 10;
	
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	std::cout << "K nearest neighbor search at (" << searchPoint.x 
	<< " " << searchPoint.y 
	<< " " << searchPoint.z
	<< ") with K=" << K << std::endl;
	
	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
			<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
			<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}
	
	// Neighbors within radius search
	
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	
	float radius = 256.0f * rand () / (RAND_MAX + 1.0);
	
	std::cout << "Neighbors within radius search at (" << searchPoint.x 
	<< " " << searchPoint.y 
	<< " " << searchPoint.z
	<< ") with radius=" << radius << std::endl;
	
	
	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	
	
	

	
}

//--------------------------------------------------------------
void kdtreeSearchExample::update(){

}

//--------------------------------------------------------------
void kdtreeSearchExample::draw(){
	//ofDrawBitmapString("HIT SPACEBAR TO SAVE VTK", 20, 20);
}

//--------------------------------------------------------------
void kdtreeSearchExample::keyPressed(int key){
	
}

//--------------------------------------------------------------
void kdtreeSearchExample::keyReleased(int key){

}

//--------------------------------------------------------------
void kdtreeSearchExample::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void kdtreeSearchExample::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void kdtreeSearchExample::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void kdtreeSearchExample::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void kdtreeSearchExample::windowResized(int w, int h){

}

//--------------------------------------------------------------
void kdtreeSearchExample::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void kdtreeSearchExample::dragEvent(ofDragInfo dragInfo){ 

}