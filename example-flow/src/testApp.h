#pragma once

#include "ofMain.h"

#include "ofxAutoControlPanel.h"

#include "ofxCv.h"
using namespace ofxCv;
using namespace cv;

class testApp : public ofBaseApp{
public:
	void setup();
	void update();
	void draw();
	void addPoint(ofVec2f& start, float z);
	
	ofImage prev, next;
	
	vector<Point2f> prevPts, nextPts;
	
	ofVideoPlayer video;
	
	bool needToAdd;
	ofMesh cloud;
	ofEasyCam camera;
	
	ofxAutoControlPanel panel;
};

