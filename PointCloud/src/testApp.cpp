#include "testApp.h"

const float FovX = 1.0144686707507438;
const float FovY = 0.78980943449644714;
const float XtoZ = tanf(FovX / 2) * 2;
const float YtoZ = tanf(FovY / 2) * 2;
const unsigned int Xres = 640;
const unsigned int Yres = 480;

ofVec3f ConvertProjectiveToRealWorld(float x, float y, float z) {
	return ofVec3f((x / Xres - .5f) * z * XtoZ,
								 (y / Yres - .5f) * z * YtoZ,
								 z);
}

void testApp::setup() {
	ofSetVerticalSync(true);
	kinect.init(false, false);  // disable infrared/rgb video iamge (faster fps)
	kinect.open();
}

void testApp::update() {
	kinect.update();
}

void testApp::draw() {
	ofBackground(0);
	
	ofSetColor(255);
	kinect.drawDepth(0, 0, 400, 300);
	
	easyCam.begin();
	int width = kinect.getWidth();
	int height = kinect.getHeight();
	ofScale(1, -1, -1); // orient the point cloud properly
	ofTranslate(0, 0, -150); // rotate about z = 150 cm
	float* distancePixels = kinect.getDistancePixels(); // distance in centimeters
	ofMesh cloud;
	cloud.setMode(OF_PRIMITIVE_POINTS);
	for(int y = 0; y < height; y++) {
		for(int x = 0; x < width; x++) {
			int i = y * width + x;
			float z = distancePixels[i];
			if(z != 0) { // ignore empty depth pixels
					cloud.addVertex(ConvertProjectiveToRealWorld(x, y, z));
			}
		}
	}
	cloud.drawVertices();
	easyCam.end();
}

void testApp::exit() {
	kinect.close();
}

void testApp::keyPressed(int key) {
}