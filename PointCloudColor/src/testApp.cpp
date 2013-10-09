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
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetVerticalSync(true);
	kinect.init();
	kinect.open();
	
	cout << "setting up registration" << endl;
	setupRegisterDepth();
	cout << "done" << endl;
	
	registeredDepth.resize(640 * 480);
	trackImage.allocate(640, 480, OF_IMAGE_GRAYSCALE);
}

void testApp::update() {
	kinect.update();
	if(kinect.isFrameNew()) {
		registerDepth(kinect.getRawDepthPixels(), &registeredDepth[0]);
		cloud.clear();
		cloud.setMode(OF_PRIMITIVE_POINTS);
		int width = kinect.getWidth();
		int height = kinect.getHeight();
		for(int y = 0; y < height; y++) {
			for(int x = 0; x < width; x++) {
				int i = y * width + x;
				float z = registeredDepth[i]; // in mm
				if(z != 0) { // ignore empty depth pixels
					ofColor curColor = kinect.getColorAt(x, y);
					float absDiff = 0;
					absDiff += abs((float) curColor.r - (float) trackColor.r);
					absDiff += abs((float) curColor.g - (float) trackColor.g);
					absDiff += abs((float) curColor.b - (float) trackColor.b);
					absDiff /= 3;
					trackImage.getPixels()[i] = absDiff;
					
					if(absDiff < mouseX) {
						cloud.addColor(kinect.getColorAt(x, y));
						cloud.addVertex(ConvertProjectiveToRealWorld(x, y, z));
					}
				}
			}
		}
		trackImage.update();
	}
}

void testApp::draw() {
	ofBackground(0);
	
	ofSetColor(255);
	kinect.draw(0, 0);
	kinect.drawDepth(640, 0);
	trackImage.draw(0, 480);
	
	easyCam.begin();
	ofScale(1, -1, -1); // orient the point cloud properly
	ofScale(.5, .5, .5);
	ofTranslate(0, 0, -1500); // rotate about z = 1.5 m
	cloud.drawVertices();
	easyCam.end();
}

void testApp::exit() {
	kinect.close();
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		trackColor = kinect.getColorAt(mouseX, mouseY);
	}
}