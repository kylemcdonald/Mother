#include "testApp.h"

// point cloud version
void ofSaveMesh(const ofMesh& mesh, string filename) {
	ofFile ply;
	if (ply.open(filename, ofFile::WriteOnly, true)) {
		int vertexCount =  mesh.getNumVertices();
		
		// write the header
		ply << "ply" << endl;
		ply << "format binary_little_endian 1.0" << endl;
		ply << "element vertex " << vertexCount << endl;
		ply << "property float x" << endl;
		ply << "property float y" << endl;
		ply << "property float z" << endl;
		ply << "end_header" << endl;
		
		// write all the vertices
		const vector<ofVec3f>& surface = mesh.getVertices();
		for(int i = 0; i < surface.size(); i++) {
			// write the raw data as if it were a stream of bytes
			ply.write((char*) &surface[i], sizeof(ofVec3f));
		}
	}
}

#define goodRawData(x) ((x) != 0 && (x) != 2047)

const float framerate = 30;

const float FovX = 1.0144686707507438;
const float FovY = 0.78980943449644714;
const float XtoZ = tanf(FovX / 2) * 2;
const float YtoZ = tanf(FovY / 2) * 2;
const unsigned int kinectWidth = 640;
const unsigned int kinectHeight = 480;

ofVec3f ConvertProjectiveToRealWorld(float x, float y, float z) {
	return ofVec3f((x / kinectWidth - .5f) * z * XtoZ,
								 (y / kinectHeight - .5f) * z * YtoZ,
								 z);
}

double dPlanePixelSize = 0.1042;
uint64_t nPlaneDsr = 120;
double planeDcl = 7.5; // pConfig->fEmitterDCmosDistance
int32_t paramCoeff = 4; // pConfig->nParamCoeff
int32_t constShift = 200; // pConfig->nConstShift
int32_t shiftScale = 10; // pConfig->nShiftScale

uint16_t rawToMillimeters(uint16_t raw) {
	double fixedRefX = (((double) raw - (paramCoeff * constShift)) / paramCoeff) - 0.375;
	double metric = fixedRefX * dPlanePixelSize;
	return shiftScale * ((metric * nPlaneDsr / (planeDcl - metric)) + nPlaneDsr);
}

void rawToMillimeters(uint16_t* data) {
	int n = kinectWidth * kinectHeight;
	for(int i = 0; i < n; i++) {
		data[i] = rawToMillimeters(data[i]);
	}
}

void testApp::setup() {
	ofSetDataPathRoot("../../../../../SharedData/");
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetVerticalSync(true);	
	
	lastAutoclear = false;
	
	fileLister.listDir("yemi");
	curImage.allocate(kinectWidth, kinectHeight, OF_IMAGE_GRAYSCALE);
	curImageAverage.allocate(kinectWidth, kinectHeight, OF_IMAGE_GRAYSCALE);
	curImageDifference.allocate(kinectWidth, kinectHeight, OF_IMAGE_GRAYSCALE);
	
	curRaw.resize(kinectWidth * kinectHeight);
	curRawAverage.resize(kinectWidth * kinectHeight);
	curRawDifference.resize(kinectWidth * kinectHeight);
	
	panel.setup(280, 1080);
	panel.addPanel("Files");
	panel.addToggle("play");
	panel.addToggle("autoclear", true);
	panel.addToggle("showAll", true);
	panel.addSlider("differenceThreshold", 5, 0, 100, true);
	panel.addFileLister("scans", &fileLister, 220, 200);
	float boxRange = 1000;
	panel.addSlider("xmin", -boxRange, -boxRange, +boxRange);
	panel.addSlider("xmax", +boxRange, -boxRange, +boxRange);
	panel.addSlider("ymin", -boxRange, -boxRange, +boxRange);
	panel.addSlider("ymax", +boxRange, -boxRange, +boxRange);
	panel.addSlider("zmin", -boxRange, -boxRange, +boxRange);
	panel.addSlider("zmax", +boxRange, -boxRange, +boxRange);
	panel.addSlider("xrot", 0, -180, +180);
	panel.addSlider("yrot", 0, -180, +180);
	panel.addSlider("zrot", 0, -180, +180);
	panel.addSlider("zoff", -1500, -2000, 0);
	
	panel.addPanel("difference");
	panel.addToggle("useDifference");
	panel.addSlider("lerpAverage", .25, 0, 1);
	panel.addDrawableRect("curImage", &curImage, 220, 165);
	panel.addDrawableRect("curImageAverage", &curImageAverage, 220, 165);
	panel.addDrawableRect("curImageDifference", &curImageDifference, 220, 165);
}

void testApp::loadFrame(int frame) {
	int n = curRaw.size();
	ifstream file;
	string curFilename = curDirectory.getPath(frame);
	file.open(ofToDataPath(curFilename).c_str(), ios::in | ios::binary);
	if (file.is_open()) {
		file.read(reinterpret_cast<char*>(&curRaw[0]), n * sizeof(uint16_t));
	}
	file.close();
	
	unsigned char* pixels = curImage.getPixels();
	for(int i = 0; i < n; i++) {
		pixels[i] = curRaw[i];
	}
	
	curImage.update();
}

void testApp::updateAverage() {
	int n = curRawAverage.size();
	float lerpAverage = panel.getValueF("lerpAverage");
	float invLerpAverage = 1 - lerpAverage;
	for(int i = 0; i < n; i++) {
		if(goodRawData(curRaw[i])) {
			curRawAverage[i] = curRaw[i] * lerpAverage + curRawAverage[i] * invLerpAverage;
		}
	}
	
	unsigned char* pixels = curImageAverage.getPixels();
	for(int i = 0; i < n; i++) {
		pixels[i] = curRawAverage[i];
	}
	
	curImageAverage.update();
}

void testApp::updateDifference() {
	int n = curRawDifference.size();
	for(int i = 0; i < n; i++) {
		if(goodRawData(curRawAverage[i]) && goodRawData(curRaw[i])) {
			curRawDifference[i] = abs((int) curRawAverage[i] - (int) curRaw[i]);
		} else {
			curRawDifference[i] = 0;
		}
	}
	
	unsigned char* pixels = curImageDifference.getPixels();
	for(int i = 0; i < n; i++) {
		pixels[i] = curRawDifference[i];
	}
	
	curImageDifference.update();
}

bool operator>(ofVec3f& a, ofVec3f& b) {
	return a.x > b.x && a.y > b.y && a.z > b.z;
}

void testApp::updateMesh() {
	int width = kinectWidth;
	int height = kinectHeight;
	bool useDifference = panel.getValueB("useDifference");
	int differenceThreshold = panel.getValueI("differenceThreshold");
	bool autoclear = panel.getValueB("autoclear");
	if(autoclear) {
		cloud.clear();
	} else if(lastAutoclear) {
		accumulatePosition = lastPosition;
	}
	lastAutoclear = autoclear;
	
	cloud.setMode(OF_PRIMITIVE_POINTS);
	
	bool showAll = panel.getValueB("showAll");
	
	float zoff = panel.getValueF("zoff");
	ofMatrix4x4 mat = ofMatrix4x4::newRotationMatrix(
		panel.getValueF("xrot"), ofVec3f(1,0,0),
		panel.getValueF("yrot"), ofVec3f(0,1,0),
		panel.getValueF("zrot"), ofVec3f(0,0,1));

	xmin = panel.getValueF("xmin");
	ymin = panel.getValueF("ymin");
	zmin = panel.getValueF("zmin");
	xmax = panel.getValueF("xmax");
	ymax = panel.getValueF("ymax");
	zmax = panel.getValueF("zmax");
	ofVec3f lower(xmin, ymin, zmin), upper(xmax, ymax, zmax);
	
	for(int y = 0; y < height; y++) {
		for(int x = 0; x < width; x++) {
			int i = y * width + x;
			if(goodRawData(curRaw[i]) && (useDifference ? curRawDifference[i] > differenceThreshold : true)) { // ignore empty depth pixels
				float z = rawToMillimeters(curRaw[i]);
				ofVec3f cur = ConvertProjectiveToRealWorld(x, y, z);
				cur.z += zoff;
				cur = ofMatrix4x4::transform3x3(cur, mat);
				if(cur > lower && upper > cur) {
					if(!autoclear) {
						cur.z += (lastPosition - accumulatePosition) * -20;
					}
					cloud.addColor(ofColor::white);
					cloud.addVertex(cur);
				} else if(showAll) {
					cloud.addColor(ofColor::gray);
					cloud.addVertex(cur);
				}
			}
		}
	}
}

void testApp::update() {
	if(fileLister.selectedHasChanged()) {
		curDirectory.listDir(fileLister.getSelectedPath());
		fileLister.clearChangedFlag();
		panel.setValueB("play", true);
	}
	
	if(panel.getValueB("play")) {
		float curTime = ofGetElapsedTimef();
		if(panel.hasValueChanged(variadic("play"))) {
			startTime = curTime;
		}
		if(curDirectory.size() > 0) {
			int curPosition = ((int) ((curTime - startTime) * framerate)) % curDirectory.size();
			if(curPosition != lastPosition) {
				loadFrame(curPosition);
				updateAverage();
				updateDifference();
				updateMesh();
			}
			lastPosition = curPosition;
		}
	}
	
	panel.clearAllChanged();
}

void cornerBox(float xmin, float ymin, float zmin, float xmax, float ymax, float zmax) {
	ofPushMatrix();
	ofTranslate(xmin, ymin, zmin);
	ofScale(xmax - xmin, ymax - ymin, zmax - zmin);
	ofTranslate(.5, .5, .5);
	ofBox(1);
	ofPopMatrix();
}

void testApp::draw() {
	ofBackground(0);
	
	easyCam.begin();
	ofScale(1, -1, -1); // orient the point cloud properly
	
	ofSetColor(ofColor::fromHex(0xec008c));
	ofNoFill();
	cornerBox(xmin, ymin, zmin, xmax, ymax, zmax);
	
	ofSetColor(255);
	glEnable(GL_DEPTH_TEST);
	cloud.drawVertices();
	glDisable(GL_DEPTH_TEST);
	
	easyCam.end();
}

void testApp::exit() {
}

void testApp::keyPressed(int key) {
	if(key == '\t') {
		fileLister.refreshDir();
	}
	if(key == 's') {
		ofSaveMesh(cloud, fileLister.getSelectedName() + ".ply");
	}
}