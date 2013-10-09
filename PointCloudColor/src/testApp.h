#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "RegisterDepth.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();
	
	void keyPressed(int key);
	
	ofxKinect kinect;
	vector<uint16_t> registeredDepth;
	ofMesh cloud;
	
	ofImage trackImage;
	
	ofColor trackColor;
	
	ofEasyCam easyCam;
};
