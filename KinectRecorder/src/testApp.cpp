#include "testApp.h"

const float mbPerFrame = 0.5859375;

template <class T>
void saveRaw(const T* obj, string filename, unsigned long size) {
	ofstream file;
	file.open(ofToDataPath(filename).c_str(), ios::out | ios::binary);
	if (file.is_open()) {
		file.write(reinterpret_cast<const char*>(obj), size);
	} else {
		ofLogError() << "Couldn't open file " << filename;
	}
	file.close();
}

void drawHighlightString(string text, int x, int y, ofColor background = ofColor::black, ofColor foreground = ofColor::white) {
	int textWidth =  10 + text.length() * 8;
	ofSetColor(background);
	ofFill();
	ofRect(x - 5, y - 12, textWidth, 20);
	ofSetColor(foreground);
	ofDrawBitmapString(text, x, y);
}

void testApp::setup() {
	ofSetDataPathRoot("../../../../../SharedData/");
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetVerticalSync(true);
	
	recording = false;
	
	kinect.init(false, false);
	kinect.open();
}

void testApp::update() {
	kinect.update();
	if(kinect.isFrameNew()) {
		if(recording) {
			unsigned long a = ofGetSystemTime();
			int n = kinect.getWidth() * kinect.getHeight();
			saveRaw(kinect.getRawDepthPixels(), curDirectoryPath + ofToString(100000 + totalFrames) + ".raw", n * sizeof(unsigned short));
			unsigned long b = ofGetSystemTime();
			//cout << n << " bytes, " << b - a << endl;
			totalFrames++;
		}
	}
}

void testApp::draw() {
	ofBackground(0);
	
	ofSetColor(255);
	kinect.drawDepth(0, 0);
	
	if(recording) {
		float curTime = ofGetElapsedTimef();
		if(fmodf(curTime, .3) < .2) {
			ofSetColor(255, 0, 0);
		} else {
			ofSetColor(0);
		}
		ofCircle(0, ofGetHeight(), 80);
		float recordingLength = curTime - startTime;
		drawHighlightString(ofToString(recordingLength, 1) + "s", 10, 20);
		drawHighlightString(ofToString(totalFrames * mbPerFrame, 2) + "MB", 10, 40);
		drawHighlightString(curDirectoryPath, 10, 60);		
	}
}

void testApp::exit() {
	kinect.close();
}

void testApp::startRecording() {
	startTime = ofGetElapsedTimef();
	curDirectoryPath = "scans/" + ofGetTimestampString("%H.%M.%S") + "/";
	if(!ofDirectory::createDirectory(curDirectoryPath, true, true)) {
		ofLogError("Failed to create " + curDirectoryPath);
		ofExit(0);
	} else {
		ofLogVerbose() << "Created " << curDirectoryPath;
	}
	totalFrames = 0;
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		recording = !recording;
		if(recording) {
			startRecording();
		}
	}
}