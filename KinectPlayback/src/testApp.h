#pragma once

#include "ofMain.h"
#include "ofxAutoControlPanel.h"

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();
	
	void loadFrame(int frame);
	void updateAverage();
	void updateDifference();
	void updateMesh();
	
	void keyPressed(int key);
	
	ofEasyCam easyCam;
	ofxAutoControlPanel panel;
	simpleFileLister fileLister;
	
	ofDirectory curDirectory;
	float startTime;
	int lastPosition;
	
	vector<uint16_t> curRaw, curRawAverage, curRawDifference;
	ofImage curImage, curImageAverage, curImageDifference;
	
	ofMesh cloud;
	float xmax, ymax, zmax, xmin, ymin, zmin;
	int accumulatePosition;
	bool lastAutoclear;
};
