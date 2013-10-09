#include "ofApp.h"

void ofSaveMesh(const ofMesh& mesh, string filename) {
	ofFile ply;
	if (ply.open(filename, ofFile::WriteOnly, true)) {	
		int vertexCount =  mesh.getNumVertices();
		int triangleCount = mesh.getNumVertices() / 3;
		cout << "saving mesh with " << mesh.getNumIndices() << " indices" << endl;
		cout << "saving mesh with " << mesh.getNumVertices() << " vertices" << endl;
		
		// write the header
		ply << "ply" << endl;
		ply << "format binary_little_endian 1.0" << endl;
		ply << "element vertex " << vertexCount << endl;
		ply << "property float x" << endl;
		ply << "property float y" << endl;
		ply << "property float z" << endl;
		ply << "element face " << triangleCount << endl;
		ply << "property list uchar int vertex_index" << endl;
		ply << "end_header" << endl;
		
		// write all the vertices
		const vector<ofVec3f>& surface = mesh.getVertices();
		for(int i = 0; i < surface.size(); i++) {
			// write the raw data as if it were a stream of bytes
			ply.write((char*) &surface[i], sizeof(ofVec3f));
		}
		
		// write all the faces
		unsigned char faceSize = 3;
		for(int i = 0; i < vertexCount; i += faceSize) {
			// write the raw data as if it were a stream of bytes
			ply.write((char*) &faceSize, sizeof(unsigned char));
			for(int j = 0; j < faceSize; j++) {
				int curIndex = i + j;
				ply.write((char*) &curIndex, sizeof(int));
			}
		}
	}
}

void ofLoadMesh(ofMesh& mesh, string filename) {
}

// first pass initializes the energy at every surface cell, and lerps the positions
// second pass analyzes all cubes (or a list of relevant cubes) and adds triangles to mesh

// switch to using a cell structure
// make it sparse -- how many of each type?
// run against old hand .ply
// optimize the mesh like aiProcess_JoinIdenticalVertices
// meshing should be the final step, each cell stores indices and some vertices
// iterative refinement of lerped vertices

// 40 seconds at 64^3 for 32k particles (naive)
// .5 ms after optimization

void getBounds(ofMesh& mesh, ofVec3f& lower, ofVec3f& upper) {
	for(int i = 0; i < mesh.getNumVertices(); i++) {
		ofVec3f& cur = mesh.getVertices()[i];
		if(i == 0) {
			lower = cur;
			upper = cur;
		} else {
			lower.x = MIN(lower.x, cur.x);
			lower.y = MIN(lower.y, cur.y);
			lower.z = MIN(lower.z, cur.z);
			upper.x = MAX(upper.x, cur.x);
			upper.y = MAX(upper.y, cur.y);
			upper.z = MAX(upper.z, cur.z);
		}
	}
}

void ofApp::setup() {
	ofSetDataPathRoot("../../../../../SharedData/");
	ofSeedRandom(0);
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	ofSetVerticalSync(true);
	
	int resolution = 256;
	mt.setup(resolution);
	mt.setRadius(1./resolution, 3./resolution);
	
	vector<ofVec3f> centers;
	
	model.loadModel("19.19.31-reduced.ply");//"malang-still-notime.ply");//
	ofMesh mesh = model.getMesh(0);
	cout << "input has " << mesh.getNumVertices() << " vertices in mesh 0, " << model.getNumMeshes() << " meshes total" << endl;
	
	ofVec3f lower, upper;
	getBounds(mesh, lower, upper);
	ofVec3f range = upper - lower;
	float maxDimension = MAX(range.x, MAX(range.y, range.z));
	float padding = .1;
	float scale = (1 - 2 * padding) / maxDimension;
	ofVec3f offset = -lower + padding * range;
	
	for(int i = 0; i < mesh.getNumVertices(); i++) {
		ofVec3f cur = mesh.getVertex(i);
		centers.push_back((cur + offset) * scale);
	}
	
	/*
	int n = 48;
	for(int y = 0; y < n; y++) {
		for(int x = 0; x < n; x++) {
			float z = (sin(x*TWO_PI) + cos(y*TWO_PI)) * ofSignedNoise(x, y) * .1 + .5;
			centers.push_back(ofVec3f(float(x) / n, float(y) / n, z));
		}
	}
	for(int i = 0; i < 0; i++) {
		centers.push_back(ofVec3f(ofRandomuf(), ofRandomuf(), ofRandomuf()));
	}
	*/
	
	mt.setCenters(centers);
	mt.update();
	
	ofSaveMesh(mt.getMesh(), "mt.ply");
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_FLAT);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	
	//cam.init();
	ofSetSphereResolution(8);
}  

void ofApp::update() {
/*
	mt.setRadius(
		ofMap(mouseX, 0, ofGetWidth(), .01, 1, true),
		ofMap(mouseY, 0, ofGetHeight(), .02, 1, true));
	mt.update();
	*/
}

void ofApp::draw() {
	ofBackground(0);
	light.enable();
	light.setPosition(0, 0, 1024);
	cam.begin();
	ofPushMatrix();
	ofScale(512,512,512);
	ofSetColor(255);
	mt.draw();
	ofPopMatrix();
	cam.end();
}

void ofApp::keyPressed(int key) {
	if(key == ' ') {
		mt.update();
	}
}