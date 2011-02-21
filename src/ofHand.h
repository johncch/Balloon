/*
 *  ofHand.h
 *  ofxKinect
 *
 *  Created by johncch on 2/20/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ofMain.h"

#ifndef _OFHAND
#define _OFHAND

typedef enum {
	LEFT_HAND = 1,
	RIGHT_HAND = -1,
} orientation;

typedef enum {
	HAND_OPEN = -2,
	HAND_CLOSE = 2
} stretch;

class ofHand {
	public:
		orientation o;
		stretch s;
	
		int lifetime;
		ofPoint thumb;
		ofPoint index;
		ofPoint bird;
		ofPoint ring;
		ofPoint pinkie;
		ofPoint centroid;
		ofRectangle boundingRect;
		ofRectangle bobRect;
		bool bobRectInit;
	
		ofHand() {
			lifetime = 10;
			s = HAND_OPEN;
			bobRectInit = false;
		};
	
		float overlap(ofHand b) {
			// ofRectangle aBounds = a.boundingRect;
			// ofRectangle bBounds = b.boundingRect;
			return overlapRect(b.boundingRect);
		};
	
		void maintainPosition(ofRectangle nBounds) {
			if(nBounds.x < boundingRect.x) {
				boundingRect.x = nBounds.x;
			}
			if(nBounds.y < boundingRect.y) {
				boundingRect.y = nBounds.y;
			}
			if(nBounds.x + nBounds.width > boundingRect.x + boundingRect.width) {
				boundingRect.x = nBounds.x + nBounds.width - boundingRect.width;
			}
			if(nBounds.y + nBounds.height > boundingRect.y + boundingRect.height) {
				boundingRect.y = nBounds.y + nBounds.height - boundingRect.height;
			}
		};
	
		float overlapRect(ofRectangle bBounds) {
			if(bBounds.x > boundingRect.x + boundingRect.width) {
				printf("returning 0\n");
				return 0;
			} else if (bBounds.y > boundingRect.y + boundingRect.height) {
				printf("returning 0\n");
				return 0;
			}
			
			float x1 = (boundingRect.x > bBounds.x) ? boundingRect.x : bBounds.x;
			float y1 = (boundingRect.y > bBounds.y) ? boundingRect.y : bBounds.y;
			float w = (boundingRect.x + boundingRect.width > bBounds.x + bBounds.width) ? bBounds.x + bBounds.width - x1 : boundingRect.x + boundingRect.width - x1;
			float h = (boundingRect.y + boundingRect.height > bBounds.y + bBounds.height) ? bBounds.y + bBounds.height - y1 : boundingRect.y + boundingRect.height - y1 ;
			float aArea = boundingRect.width * boundingRect.height;
			// float bArea = bBounds.width * bBounds.height;
			float area = w * h;
			// return area / (aArea + bArea - area) > threshold;
			// return area / aArea > threshold;
			printf("overlap ratio is %f\n", area/aArea);
			return area / aArea;
		};
	
		float containsRect(ofRectangle bBounds, float threshold=0.9) {
			if(bBounds.x > boundingRect.x + boundingRect.width) {
								printf("returning 0\n");
				return 0;
			} else if (bBounds.y > boundingRect.y + boundingRect.height) {
								printf("returning 0\n");
				return 0;
			}
			
			float x1 = (boundingRect.x > bBounds.x) ? boundingRect.x : bBounds.x;
			float y1 = (boundingRect.y > bBounds.y) ? boundingRect.y : bBounds.y;
			float w = (boundingRect.x + boundingRect.width > bBounds.x + bBounds.width) ? bBounds.x + bBounds.width - x1 : boundingRect.x + boundingRect.width - x1;
			float h = (boundingRect.y + boundingRect.height > bBounds.y + bBounds.height) ? bBounds.y + bBounds.height - y1 : boundingRect.y + boundingRect.height - y1 ;
			// float aArea = boundingRect.width * boundingRect.height;
			float bArea = bBounds.width * bBounds.height;
			float area = w * h;
			// return area / (aArea + bArea - area) > threshold;
			// return area / aArea > threshold;
			printf("contains ratio is %f because %f %f\n", area/bArea, area, bArea);
			return (area / bArea);
		}
	
		void copyAttributes(ofHand b) {
			thumb = b.thumb;
			index = b.index;
			bird = b.bird;
			ring = b.ring;
			pinkie = b.pinkie;
			centroid = b.centroid;
			boundingRect = b.boundingRect;
			// lifetime = b.lifetime;
		}
	
		void addLife() {
			if (lifetime < 60) {
				lifetime+= 2;
			}
			s = HAND_OPEN;
		}
	
		void maintainLife() {
			lifetime += 1;
			s = HAND_CLOSE;
		}
	
		void reduceLife() {
			if(lifetime > 0) {
				lifetime --;
			}
		}
};

#endif
