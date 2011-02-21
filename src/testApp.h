#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofHand.h"

#define PEAK_NUM 10
#define BLOCK_SIDE 2
#define MAX_DEPTH 600
#define MAX_DEPTH_TURNAROUND 500
#define THRESHOLD_STEP_SIZE 5

#define UPPER_SEARCH_RATIO 0.9
#define LOWER_SEARCH_RATIO 0.05

#define SEARCH_NEAR 250
#define SEARCH_FAR 180
#define SEARCH_RANGE 13
#define SEARCH_BACKTRACK 3

#define FIST_AREA_MIN 1000
#define HAND_AREA_MIN 4000
#define HAND_AREA_MAX 9000

#define HAND_RATIO_MIN 0.35
#define HAND_RATIO_MAX 0.62

#define MAXIMA_THRESHOLD 4

struct PointCloudPoint {
	int x;
	int y;
	float val;
};

class testApp : public ofBaseApp {
	private:
		// std::vector<PointCloudPoint> peaks;
		bool run;
		int movingTarget;
		int stepSize;
		int threshold;
	
		int thisFrameNumberBlobs;
		stringstream stats;
		
		vector<ofHand> newHands;
		vector<ofHand> hands;
	
		float* distancePixels;
		unsigned char* depthPixels;
		// unsigned char* depthPixelsDx;
		// unsigned char* depthPixelsDy;
	
		// ofxCvGrayscaleImage dyGrayImage;
		// ofxCvGrayscaleImage dxGrayImage;
	
	public:

		int width;
		int height;
	
		void setup();
		void update();
		void draw();
		void exit();
	
		// void drawPointCloud();

		void keyPressed  (int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		float calculateDistance(ofPoint a, ofPoint b);

		ofxKinect kinect;

		ofxCvColorImage		colorImg;

		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage		thresholdImage;
		ofxCvGrayscaleImage		totalThresholdImage;
		// ofxCvGrayscaleImage 	grayThresh;
		// ofxCvGrayscaleImage 	grayThreshFar;

		ofxCvContourFinder 	contourFinder;
		
		// bool				bThreshWithOpenCV;
		// bool				drawPC;

		int 				nearThreshold;
		int					farThreshold;

		int					angle;
		
		// int 				pointCloudRotationY;
};
