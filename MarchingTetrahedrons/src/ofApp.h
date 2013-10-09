#pragma once

#include "ofMain.h"
#include "MarchingTetrahedrons.h"
#include "ofxAssimpModelLoader.h"

class ofApp : public ofBaseApp {
public:
	void setup(), update(), draw();
	void keyPressed(int key);
	
	ofEasyCam cam;
	ofLight light;
	
	ofxAssimpModelLoader model;
	
	MarchingTetrahedrons mt;
};
