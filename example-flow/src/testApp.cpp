#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(25);
	
	video.loadMovie("fleur-crop.mov");
	video.play();
	
	prev.allocate(video.getWidth(), video.getHeight(), OF_IMAGE_GRAYSCALE);
	next.allocate(video.getWidth(), video.getHeight(), OF_IMAGE_GRAYSCALE);
	
	panel.setup(250, 700);
	
	panel.addPanel("Optical Flow");
	
	panel.addSlider("winSize", 16, 4, 64, true);
	panel.addSlider("maxLevel", 2, 0, 8, true);
	
	panel.addSlider("maxFeatures", 300, 1, 1000);
	panel.addSlider("qualityLevel", 0.01, 0.001, .02);
	panel.addSlider("minDistance", 1, 1, 16);
	
	panel.addSlider("minLength", 6, 0, 64);
	panel.addSlider("maxLength", 16, 1, 64);
	
	panel.addSlider("jumpTo", 0, 0, 1);
	panel.addSlider("timeScale", 10000, 100, 100000);
	
	needToAdd = false;
	cloud.setMode(OF_POINTS_MODE);
}

void testApp::update(){
	if(panel.hasValueChanged(variadic("jumpTo"))) {
		video.setPosition(panel.getValueF("jumpTo"));
		panel.clearAllChanged();
	} else {
		panel.setValueF("jumpTo", video.getPosition());
	}
	
	video.update();
	
	if(video.isFrameNew()) {
		prev.setFromPixels(next.getPixels(), next.getWidth(), next.getHeight(), OF_IMAGE_GRAYSCALE);
		next.setFromPixels(video.getPixels(), video.getWidth(), video.getHeight(), OF_IMAGE_COLOR);
		next.setImageType(OF_IMAGE_GRAYSCALE);
		
		prevPts.clear();
		goodFeaturesToTrack(toCv(prev),
												prevPts,
												panel.getValueI("maxFeatures"),
												panel.getValueF("qualityLevel"),
												panel.getValueF("minDistance"));
		
		vector<uchar> status;
		vector<float> err;
		int winSize = panel.getValueI("winSize");
		calcOpticalFlowPyrLK(toCv(prev),
												 toCv(next),
												 prevPts,
												 nextPts,
												 status,
												 err,
												 
												 cv::Size(winSize, winSize),
												 panel.getValueI("maxLevel")
												 );
		
		needToAdd = true;
	}
}

void testApp::addPoint(ofVec2f& start, float z) {
	cloud.addVertex(ofVec3f(start.x, start.y, z));
}

void testApp::draw(){
	ofBackground(0);
	
	ofPushMatrix();
	
	ofSetColor(255);
	ofTranslate(280, 0);
	prev.draw(0, 0);
	next.draw(prev.getWidth(), 0);
	
	float minLength = panel.getValueF("minLength");
	float maxLength = panel.getValueF("maxLength");
	
	ofVec2f average;
	for(int i = 0; i < prevPts.size(); i++) {
		ofVec2f start = toOf(prevPts[i]);
		ofVec2f end = toOf(nextPts[i]);
		average += (end - start);
	}
	average /= prevPts.size();
	
	float position = video.getPosition();
	
	for(int i = 0; i < prevPts.size(); i++) {
		ofVec2f start = toOf(prevPts[i]);
		ofVec2f end = toOf(nextPts[i]);
		ofVec2f diff = (end - start) - average;
		float dist = diff.length();
		if(dist > minLength && dist < maxLength) {
			ofSetColor(ofColor::cyan);
			if(needToAdd) {
				addPoint(start, position);
			}
		} else {
			ofSetColor(ofColor::white);
		}
		ofPushMatrix();
		ofTranslate(start.x, start.y);
		ofLine(0, 0, diff.x, diff.y);
		ofPopMatrix();
	}
	needToAdd = false;
	
	ofTranslate(prev.getWidth() / 2, next.getWidth() / 2);
	ofSetColor(ofColor::yellow);
	ofLine(0, 0, average.x * 100, average.y * 100);
	
	ofPopMatrix();
	
	camera.begin();
	ofSetColor(255);
	ofScale(1, -1, panel.getValueF("timeScale"));
	ofTranslate(-video.getWidth() / 2, -video.getHeight() / 2, -video.getPosition());
	cloud.drawVertices();
	camera.end();
}