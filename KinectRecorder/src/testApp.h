#pragma once

#include "ofMain.h"
#include "ofxKinect.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();
	
	void startRecording();
	
	void keyPressed(int key);
	
	ofxKinect kinect;
	bool recording;
	float startTime;
	int totalFrames;
	string curDirectoryPath;
};
