#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetFrameRate(60);
	ofSetLogLevel(OF_LOG_VERBOSE);	
	
	// kinect.init(true);  //shows infrared image
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	
	width = kinect.width;
	height = kinect.height;
	
	ofSetWindowShape(kinect.width * BLOCK_SIDE, kinect.height * BLOCK_SIDE);	
	
	grayImage.allocate(kinect.width, kinect.height);
	thresholdImage.allocate(kinect.width, kinect.height);
	totalThresholdImage.allocate(width, height);
	
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	nearThreshold = SEARCH_NEAR;
	farThreshold  = SEARCH_NEAR - SEARCH_RANGE;
	
	run = true;
	movingTarget = 1;
	stepSize = 1;
	threshold = 100;
	
//	int lengthOfDx = kinect.height * (kinect.width - 1);
//	depthPixelsDx = new unsigned char[lengthOfDx];
//	int lengthOfDy = (kinect.height - 1) * kinect.width;
//	depthPixelsDy = new unsigned char[lengthOfDy];
	
//	ofLog(OF_LOG_VERBOSE, str.str());
//	printf(str);
//	printf("%d, %d\n", kinect.width, kinect.height);

	
//	colorImg.allocate(kinect.width, kinect.height);
	
//	dxGrayImage.allocate(kinect.width - 1, kinect.height);
//	dyGrayImage.allocate(kinect.width, kinect.height - 1);
//	grayThresh.allocate(kinect.width, kinect.height);
//	grayThreshFar.allocate(kinect.width, kinect.height);
//
//	bThreshWithOpenCV = true;
		
	// zero the tilt on startup
	
	
	// start from the front
	// pointCloudRotationY = 180;
	
	// drawPC = false;
}

//--------------------------------------------------------------
void testApp::update() {		
	if(run){

//		movingTarget += stepSize;
//		if(movingTarget > MAX_DEPTH_TURNAROUND) {
//			movingTarget = 1;
//		}			
		kinect.update();
		if(kinect.isFrameNew())	{// there is a new frame and we are connected		
			//distancePixels = kinect.getDistancePixels();
			depthPixels = kinect.getDepthPixels();
			grayImage.setFromPixels(depthPixels, width, height);
			
			newHands.clear();			
			stats.str("");
			
			while (farThreshold > SEARCH_FAR) {
			//	printf("searching from far %d to %d\n", farThreshold, nearThreshold);
				thresholdImage.setFromPixels(depthPixels, width, height);
				stats << "Searching from " << farThreshold << " to " << nearThreshold << endl;
			
				unsigned char* pix = thresholdImage.getPixels();
				int numPixels = thresholdImage.getWidth() * grayImage.getHeight();
			
				for(int i = 0; i < numPixels; i++){
					if( pix[i] < nearThreshold && pix[i] > farThreshold ){
						pix[i] = 255;
					} else {
						pix[i] = 0;
					}
				}
				thresholdImage.flagImageChanged();			
				contourFinder.findContours(thresholdImage, FIST_AREA_MIN, HAND_AREA_MAX, 20, false);
				
				// cull blobs
				vector<ofxCvBlob> blobs = contourFinder.blobs;
				stats << "Detected " << blobs.size() << " number of blobs" << endl;
			//	printf("number of blobs before %d\n", (int) blobs.size());
				for (int i = 0; i < blobs.size(); i++) {
					ofRectangle r = blobs[i].boundingRect;
					if (r.y < height * LOWER_SEARCH_RATIO) {
						stats << "Dropping blob on the top" << endl;
						blobs.erase(blobs.begin() + i);
					} else if (r.y + r.height > height * UPPER_SEARCH_RATIO) {
						stats << "Dropping blob at the bottom" << endl;
						blobs.erase(blobs.begin() + i);
					}
				}
			//	printf("number of blobs after %d\n", (int) blobs.size());
				
				// Now we process the blobs
				for (int i = 0; i < blobs.size(); i++) {
					ofxCvBlob blob = blobs[i];
					ofRectangle r = blob.boundingRect;
					
					if(blob.area > HAND_AREA_MIN) {					
						float ratio = (blob.area) / (r.width * r.height);
						if( ratio > HAND_RATIO_MAX || ratio < HAND_RATIO_MIN ) {
							// not likely to be hand;
				//			printf("ratio is %f, Not hand, discarding..\n", ratio);
							stats << "ratio is " << ratio << ", not hands and discarding" << endl;
							continue;
						}
						
						// create new hand
						ofHand h;
						h.centroid = blob.centroid;
						h.boundingRect = r;
						
						// calculate distance from centroid
						float maxDistance = 0;
						float minDistance = 100;
						vector<float> distance;
						for (int j = 0; j < blob.nPts; j ++) {
							float newDistance = calculateDistance(blob.pts[j], blob.centroid);
							distance.push_back(newDistance);
							if(newDistance > maxDistance) maxDistance = newDistance;
							if(newDistance < minDistance) minDistance = newDistance;
						}
						
						// float distThreshold = (maxDistance) / 2;
						float distThreshold = r.width / 2;
						
						// get maximas
						vector<ofPoint> points;
						vector<ofPoint> maximas;
						int startJ = 0;
						bool overThresh = false;
						bool startWithOverThresh = (distance[0] > distThreshold);
						for (int j = 0; j < blob.nPts; j++) {
							float dist = distance[j];
							if(!overThresh) {
								if(dist > distThreshold) {
									overThresh = true;
									startJ = j;
								}
							} else {
								if(dist < distThreshold) {
									overThresh = false;
									float x, y;
									int maxIndex = 0;
									int maxDistance = 0;
									for	(int k = 0; k < points.size(); k++) {
										if(distance[startJ + k] > maxDistance) {
											maxIndex = startJ + k;
											maxDistance = distance[maxIndex];
										}
									}
									maximas.push_back(blob.pts[maxIndex]);
									points.clear();
								}
							}
							if(overThresh) {
								points.push_back(blob.pts[j]);	
							}
						}
						
						if(maximas.size() < MAXIMA_THRESHOLD ) {
							stats << "The size of maximas is " << maximas.size() << " discarding... " << endl;
							continue;
						} else {
							stats << "The size of maximas is " << maximas.size() << " keeping..." << endl;
						}
						
						int maxIndex = 0;
						int maxFDistance = 0;
						int fDistance = 0;
						for(int j = 0; j < maximas.size(); j++) {
							ofPoint fingerPoint = maximas[j];
							for (int k = 0; k < maximas.size(); k++) {
								if (j == k) continue;
								fDistance += calculateDistance(fingerPoint, maximas[k]);
							}
							if(fDistance > maxFDistance) {
								maxFDistance = fDistance;
								maxIndex = j;
							}
						}
						// printf("size of maximas is %d and maxIndex is %d", (int) maximas.size(), maxIndex);
						h.thumb = maximas[maxIndex];
						
						// newHands.push_back(*h);
						
						bool added = false;
						for (int j = 0; j < hands.size(); j++) {
							stats << "Getting " << hands[j].overlap(h) << endl;
							if(hands[j].overlap(h) > 0.5) {
								hands[j].copyAttributes(h);
								hands[j].addLife();
								added = true;
								break;
							}
						}
						if(!added) {
							stats << "h x is " << h.boundingRect.x << " and y is " << h.boundingRect.y;
							stats << " and width is " << h.boundingRect.width << " and height is " << h.boundingRect.height << endl;
							hands.push_back(h);
						}
						
						maximas.clear();
					} else {
						stats << "Detected a small blob" << endl;
						for (int j = 0; j < hands.size(); j++) {
						/*	stats << "hands x " << hands[j].boundingRect.x << " and y " << hands[j].boundingRect.y;
							stats << " width " << hands[j].boundingRect.width << " height " << hands[j].boundingRect.height << endl;
							stats << " and blob x " << blob.boundingRect.x << " and blob y " << blob.boundingRect.y;
							stats << " width " << blob.boundingRect.width << " and height " << blob.boundingRect.height << endl;
							stats << "overlap is " << hands[j].overlapRect(blob.boundingRect) << endl;
							stats << "contain is " << hands[j].containsRect(blob.boundingRect) << endl;*/
							ofSetColor(255, 0, 0);
							ofRect(blob.boundingRect.x, blob.boundingRect.y, blob.boundingRect.width, blob.boundingRect.height);
							if(hands[j].containsRect(blob.boundingRect) > 0.8) {
								hands[j].maintainPosition(blob.boundingRect);
								hands[j].maintainLife();
								stats << "maintaining blob life" << endl;
							}
						}
					}
				}
				
				nearThreshold -= SEARCH_RANGE - SEARCH_BACKTRACK;
				farThreshold -= SEARCH_RANGE - SEARCH_BACKTRACK;				
				stats << endl;				
				
			} // end while
			
			for(int i = 0; i < hands.size(); i++) {
				hands[i].reduceLife();
				if(hands[i].lifetime <= 0) {
					hands.erase(hands.begin() + i);
				}
			}
			
			// replicate and cull
			
//			for (int i = 0; i < hands.size(); i++) {
//				bool oHandReplicated = false;
//				for (int j = 0; j < newHands.size(); j++) {
//					ofHand nHand = newHands[j];
//					stats << "I'm getting overlap " << hands[i].overlap(nHand) << endl;
//					if (hands[i].overlap(nHand) > 0.5) {
//						hands[i].copyAttributes(nHand);
//						stats << "adding .. " << hands[i].lifetime << endl;
//						hands[i].addLife();
//						newHands.erase(newHands.begin() + j);
//						oHandReplicated = true;
//						break;
//					}
//				}
//				if (!oHandReplicated) {
//					hands[i].reduceLife();
//				}
//			}
//			
//			for (int i = 0; i < hands.size(); i++) {
//				if(hands[i].lifetime <= 0) {
//					stats << "erasing" << endl;
//					hands.erase(hands.begin() + i);
//				}
//			}
//			
//			stats << newHands.size();
//			for (int i = 0; i < newHands.size(); i++) {
//				stats << "pushing" << endl;
//				hands.push_back(newHands[i]);
//			}
			
			//ofLog(OF_LOG_VERBOSE, ofToString(sizeof(distancePixels)/ sizeof(float)));
			/* peaks.clear();
			for (int i = 0; i < kinect.height; i++){
				for(int j = 0; j < kinect.width; j++) {
					int kIndex = i * kinect.width + j;
					float kPixelVal = distancePixels[kIndex];
					if (kPixelVal == 0) continue;

					PointCloudPoint p;
					p.x = j;
					p.y = i;
					p.val = kPixelVal;
					
					if(peaks.size() < PEAK_NUM) {
						peaks.push_back(p);
					} else {
						for (int x = 0; x < PEAK_NUM; x++) {
							PointCloudPoint q = peaks[x];
							if(kPixelVal < q.val) {
								peaks.erase(peaks.begin() + x);
								peaks.push_back(p);
								break;
							}
						}
					}
					//printf("%f, ", distancePixels[i * kinect.width + j]);
					
				}
			}
			
			for(int i = 0; i < kinect.height; i++){
				for(int j = 1; j < kinect.width; j++){
					int index = i * kinect.width + j;
					int dxIndex = i * (kinect.width - 1) + (j - 1);
					depthPixelsDx[dxIndex] = depthPixels[index] - depthPixels[index - 1];
				}
			}
			
			for(int i = 1; i < kinect.height; i++) {
				for (int j = 0; j < kinect.width; j++) {
					int index = i * kinect.width + j;
					int dyIndex = (i - 1) * kinect.width + j;
					depthPixelsDy[dyIndex] = depthPixels[index] - depthPixels[index - kinect.width];
				}
			}*/
					
		//we do two thresholds - one for the far plane and one for the near plane
		//we then do a cvAnd to get the pixels which are a union of the two thresholds.	
/*		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThresh.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}else{
		
			//or we do it ourselves - show people how they can work with the pixels
		
			unsigned char * pix = grayImage.getPixels();
			int numPixels = grayImage.getWidth() * grayImage.getHeight();

			for(int i = 0; i < numPixels; i++){
				if( pix[i] < nearThreshold && pix[i] > farThreshold ){
					pix[i] = 255;
				}else{
					pix[i] = 0;
				}
			}
		}

		//update the cv image
		grayImage.flagImageChanged();
	
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
    	// also, find holes is set to true so we will get interior contours as well....*/
    	
		} // end while
		
		nearThreshold = SEARCH_NEAR;
		farThreshold  = SEARCH_NEAR - SEARCH_RANGE;
					   
	} // end run
}

float testApp::calculateDistance(ofPoint a, ofPoint b) {
	float xx = a.x - b.x;
	float yy = a.y - b.y;
	return (float) sqrt(xx*xx + yy*yy);
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofBackground(0, 0, 0);
	ofSetColor(255, 255, 255);

	grayImage.draw(0, 0);
	
	stringstream out;
	for (int i = 0; i < hands.size(); i++) {
		out << "for hand " << i << endl;
		ofHand h = hands[i];
		if(h.lifetime < 30) {
			out << " not printing " << endl;
			continue;
		}
		out << " printing " << endl;
		out << "Hand is " << ((h.s == HAND_OPEN) ? "open" : "close") << endl;
		ofSetColor(0, 255, 255);
		ofNoFill();
		ofRect(h.boundingRect.x, h.boundingRect.y, h.boundingRect.width, h.boundingRect.height);
		
		ofSetColor(0, 0, 255);
		ofFill();
		ofCircle(h.thumb.x, h.thumb.y, 10);
	}
	
	totalThresholdImage.setFromPixels(depthPixels, width, height);
	
	unsigned char* pix = totalThresholdImage.getPixels();
	int numPixels = totalThresholdImage.getWidth() * totalThresholdImage.getHeight();
	
	for(int i = 0; i < numPixels; i++){
		if( pix[i] < SEARCH_NEAR && pix[i] > SEARCH_FAR){
			pix[i] = 255;
		}else{
			pix[i] = 0;
		}
	}
	totalThresholdImage.flagImageChanged();	
	totalThresholdImage.draw(width, 0);
	
	ofSetColor(0, 0, 255);
	
	out << "We have detected " << hands.size() << " hands";
	ofDrawBitmapString(out.str(), 50, 50);
	ofDrawBitmapString(stats.str(), width + 50, 50);
 	
	
	
/*	if(drawPC){
		ofPushMatrix();
		ofTranslate(420, 320);
		// we need a proper camera class
		drawPointCloud();
		ofPopMatrix();
	}else{
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);

		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
	}*/
	
	// grayImage.draw(10, 320, 400, 300);
	// grayImage.draw(0, 0);
	
	// grayImage.setFromPixels(depthPixels, kinect.width, kinect.height);
	// dxGrayImage.setFromPixels(depthPixelsDx, kinect.width - 1, kinect.height);
	// dyGrayImage.setFromPixels(depthPixelsDy, kinect.width, kinect.height - 1);

	//dxGrayImage.draw(0, kinect.height);
//	dyGrayImage.draw(kinect.width, kinect.height);

//	glBegin(GL_POINTS);
//	glColor3ub((unsigned char) 255,(unsigned char) 0, (unsigned char) 0);
//	for(int i = 0; i < kinect.height; i++) {
//		for (int j = 0; j < kinect.width; j++) {
//
//			int index = i * kinect.width + j;
//			if (distancePixels[index] == 0) {
//				continue;
//			}
//			if ((int) distancePixels[index] >= movingTarget && (int) distancePixels[index] <= movingTarget + stepSize) {
//
//				
//				glVertex2f(j, i);
//
//
//			}
//			if(distancePixels[index] <= threshold) {
//				glVertex2f(kinect.width + j, i);
//			}
//		}
//	}
//	glEnd();

	/* unsigned char * pix = grayImage.getPixels();
	int numPixels = grayImage.getWidth() * grayImage.getHeight();
	
	for(int i = 0; i < numPixels; i++){
		if( pix[i] < nearThreshold && pix[i] > farThreshold ){
			pix[i] = 255;
		}else{
			pix[i] = 0;
		}
	}
	grayImage.flagImageChanged();
	
	/*grayThreshFar = grayImage;
	grayThresh = grayImage;
	grayThresh.threshold(nearThreshold, true);
	grayThreshFar.threshold(farThreshold);
	grayThresh.draw(0, kinect.height);
	grayThreshFar.draw(kinect.width, kinect.height);

	cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);	
	*/
	//	grayImage.draw(0, kinect.height);
	// contourFinder.draw(0, 0, kinect.width, kinect.height);
	
	//ofSetColor(0, 0, 255);
//	ofFill();
//	stringstream blobStats;
//
//	
//	blobStats << "We have " << blobs.size() << " of blobs" << endl;
//	for (int i = 0; i < blobs.size(); i++) {
//		blobStats << "blob " << i << ":" << endl;
//		ofxCvBlob blob = blobs[i];
//		ofRectangle r = blob.boundingRect;		
//		blobStats << "Blob is at " << r.x << ", " << r.y << endl;
//		blobStats << "Rect is " << r.width * r.height <<  " and area is " << blob.area << endl;
//		blobStats << "Blob ratio is at " << blob.area / (r.width * r.height) << endl;
//		blobStats << "There are " << blob.nPts << " in this blob" << endl;
//		ofCircle(blob.centroid.x, blob.centroid.y, 5);
		/* int var = 0;
		int distance = 0;
		for (int j = 0; j < blob.nPts; j += 20) {
			float xx = blob.pts[j].x - blob.centroid.x;
			float yy = blob.pts[j].y - blob.centroid.y;
			float newDistance = (float) sqrt(xx*xx + yy*yy);
			float diffDistance = (newDistance - distance);
			float diffDistanceSqr = sqrt(diffDistance * diffDistance);
			var += diffDistanceSqr;
			if(diffDistanceSqr > 50) {
				ofCircle(blob.pts[j].x, blob.pts[j].y, 10);
			}
		}
		blobStats << "The absolute distance is " << var << endl;*/
		
		/*vector<ofPoint> poc;
		for (int j = 0; j < blob.nPts; j++) {
			ofPoint p = blob.pts[j];
			if (p.x < r.x + 0.1 * r.width || p.x > r.x + 0.9 * r.width ) {
				poc.push_back(p);
			} else if (p.y < r.y + 0.1 * r.height || p.y > r.y + 0.9 * r.height) {
				poc.push_back(p);
			}
				
		}
		ofSetColor(0, 255, 255);
		for (int j = 0; j < poc.size(); j++){
			ofPoint p = poc[j];
			ofCircle(p.x, p.y, 1);
		}*/
		
		//float maxDistance = 0;
//		float minDistance = 100;
//		vector<float> distance;
//		for (int j = 0; j < blob.nPts; j ++) {
//			float xx = blob.pts[j].x - blob.centroid.x;
//			float yy = blob.pts[j].y - blob.centroid.y;
//			float newDistance = (float) sqrt(xx*xx + yy*yy);
//			distance.push_back(newDistance);
//			if(newDistance > maxDistance) maxDistance = newDistance;
//			if(newDistance < minDistance) minDistance = newDistance;
//		}
//		
//		float distThreshold = (maxDistance) / 2;
//		
//		vector<ofPoint> points;
//		vector<ofPoint> maximas;
//		int startJ = 0;
//		bool overThresh = false;
//		bool startWithOverThresh = (distance[0] > distThreshold);
//		for (int j = 0; j < blob.nPts; j++) {
//			float dist = distance[j];
//			if(!overThresh) {
//				if(dist > distThreshold) {
//					overThresh = true;
//					startJ = j;
//				}
//			} else {
//				if(dist < distThreshold) {
//					overThresh = false;
//					float x, y;
//					int maxIndex = 0;
//					int maxDistance = 0;
//					for	(int k = 0; k < points.size(); k++) {
//						if(distance[startJ + k] > maxDistance) {
//							maxIndex = startJ + k;
//							maxDistance = distance[maxIndex];
//						}
//					}
//					//ofPoint max; 
//					// max.set(x / points.size(), y / points.size());
//					//max.set(points[maxIndex].x, points[maxIndex].y);
//					maximas.push_back(blob.pts[maxIndex]);
//					points.clear();
//				}
//			}
//			if(overThresh) {
//				points.push_back(blob.pts[j]);	
//			}
//		}
//		
//		ofSetColor(0, 0, 255);
//		ofFill();
//		for (int j = 0; j < maximas.size(); j++) {
//			ofCircle(maximas[j].x, maximas[j].y, 10);
//		}
//		
//		for(int j = 0; j < distance.size(); j++) {
//			ofLine(20 + j, 400, 20 + j, 400 - distance[j]);
//		}
//		
//		ofSetColor(255, 255, 0);
//		ofNoFill();
//		ofRect(r.x, r.y, r.width, r.height);
//	}
//	ofDrawBitmapString(blobStats.str(), 20, 20);
	
	/*for (int i = 0; i < kinect.height; i++) {
		for(int j = 0; j < kinect.width; j++) {
			int index = i * kinect.width + j;
			int color = distancePixels[index] / MAX_DEPTH * 255;
			ofSetColor(color, color, color);
			ofRect(j * BLOCK_SIDE, i * BLOCK_SIDE, BLOCK_SIDE, BLOCK_SIDE);
		}
	}*/
	
//	ofSetColor(0, 0, 255);
//	stringstream thresholdStats;
//	thresholdStats << "Current threshold: " << threshold << endl;
//	ofDrawBitmapString(thresholdStats.str(), kinect.width + 50, 50);
//	
//	stringstream statistics;
//	statistics << "Kinect width " << kinect.width << " and height " << kinect.height << endl;
//	statistics << "Size of peak is " << peaks.size() << endl;
//	statistics << "Peak points are: " << endl;
//	
//	ofSetColor(255, 0, 0);
//	for(int i = 0; i < peaks.size(); i++) {
//		PointCloudPoint c = peaks[i];
//		ofCircle(c.x, c.y, 2);
//		ofCircle(kinect.width + c.x, kinect.height + c.y, 2);
//		ofCircle(c.x, kinect.height + c.y, 2);
//		statistics << i << ". Point at " << c.x << ", " << c.y << " is the value " << c.val << endl;
//	}
//	ofSetColor(0, 0, 255);
//	ofDrawBitmapString(statistics.str(), 20, 750);

	/*stringstream output;
	int index = (mouseY / 2) * kinect.width + (mouseX / 2);
	output << "Depth value at " << (mouseX / 2) << ", " << (mouseY / 2) << " is " << distancePixels[index];
	ofSetColor(0, 255, 0);
	ofDrawBitmapString(output.str(), mouseX, mouseY);	*/
	
	
/*	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
								 << ofToString(kinect.getMksAccel().y, 2) << " / " 
								 << ofToString(kinect.getMksAccel().z, 2) << endl
				 << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
				 << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
				 << "set near threshold " << nearThreshold << " (press: + -)" << endl
				 << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
				 	<< ", fps: " << ofGetFrameRate() << endl
				 << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
				 << "press UP and DOWN to change the tilt angle: " << angle << " degrees";
	ofDrawBitmapString(reportStream.str(),20,666);*/
	
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch(key) {
		case 'p':
			run = !run;
			break;
		case '+':
			if (stepSize < 10) {
				stepSize += 1;
			}
			break;
		case '_':
			if(stepSize > 1) {
				stepSize -= 1;
			}
			break;
		case '=':
			threshold += THRESHOLD_STEP_SIZE;
			break;
		case '-':
			if(threshold >= 40) {
				threshold -= THRESHOLD_STEP_SIZE;
			}
			break;
	}
/*	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
		break;
		case'p':
			drawPC = !drawPC;
			break;
	
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
		case 'o':
			kinect.setCameraTiltAngle(angle);	// go back to prev tilt
			kinect.open();
			break;
		case 'c':
			kinect.setCameraTiltAngle(0);		// zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
*/
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
//	pointCloudRotationY = x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

