#pragma once

#include "ofMain.h"

class IndexedSamples {
public:
	vector< pair<int, int> > samples;
	vector<unsigned int> indices;
};

class MarchingTetrahedrons {
public:
	MarchingTetrahedrons();
	void setup(int resolution);
	void update();
	void draw();
	
	void setCenters(vector<ofVec3f>& centers);
	void setRadius(float minRadius, float maxRadius);
	void drawCubes();
	
	ofMesh getMesh();
	
protected:
	ofVboMesh mesh;
	
	vector<ofVec3f> centers;
	float minRadius, maxRadius;
	
	int resolution;
	vector<float> cells;
	vector<IndexedSamples> tetrahedrons, cubes;
	vector<int> usageCount;
	
	void buildTetrahedrons(), buildCubes(), buildCells(), buildMesh();
	int getIndex(int x, int y, int z);
	ofVec3f getVertex(int x, int y, int z);
	void getPosition(int i, int& x, int& y, int& z);
	ofVec3f getVertex(int i);
};