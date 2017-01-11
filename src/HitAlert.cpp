#include "HitAlert.hpp"

HitAlert::HitAlert(Config config)
{
	config_ = config;
	Size frameSize(config_.frameWidth, config_.frameHeight);
	currentFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	previousFrame_ = Mat(frameSize, CV_8UC1, Scalar(0));
	riskMap_ = Mat(frameSize, CV_8UC1, Scalar(0));
}



void HitAlert::setCurrentFrame(const Mat& newFrame)
{
	resize(newFrame, currentFrameRaw_, Size(config_.frameWidth, config_.frameHeight));

	Mat newFrameGray;
	cvtColor(newFrame, newFrameGray, CV_BGR2GRAY);
	resize(newFrameGray, currentFrame_, Size(config_.frameWidth, config_.frameHeight));

	// TODO Preprocess?

	////Mat sharpen = (
	////	Mat_<uchar>(3, 3) <<
	////	-1, -1, -1,
	////	-1, 9, -1,
	////	-1, -1, -1
	////	);
	////filter2D(pic, pic, -1, sharpen);
}



void HitAlert::pushFrame()
{
	previousFrame_ = currentFrame_.clone();
	previousFrameRaw_ = currentFrameRaw_.clone();
}



void HitAlert::calculateRiskMap()
{
	riskMap_ = 0;
	nextPosition_.clear();
	goodFeaturesToTrack(
		previousFrame_, previousTrackedPoints_, 
		config_.trackMaxCorners, 
		config_.trackQuality,
		config_.trackMinDistance);
	if (previousTrackedPoints_.size() == 0) return;

	vector<unsigned char> status;
	vector<float> err;
	calcOpticalFlowPyrLK(
		previousFrame_, currentFrame_, 
		previousTrackedPoints_, currentTrackedPoints_,
		status, err);

	// delaunay
	optFlowMean_ = Point2f(0, 0);
	int validOptFlowCount = 0;

	Subdiv2D subdiv(Rect(0, 0, config_.frameWidth, config_.frameHeight));
	for (int i = 0; i < previousTrackedPoints_.size(); i++) {
		if (status[i]) {
			subdiv.insert(previousTrackedPoints_[i]);
			nextPosition_[previousTrackedPoints_[i]] = currentTrackedPoints_[i];
			optFlowMean_ += currentTrackedPoints_[i] - previousTrackedPoints_[i];
			validOptFlowCount++;
		}
	}
	optFlowMean_ /= validOptFlowCount;

	vector<Vec6f> trianglesRaw, triangles;
	subdiv.getTriangleList(trianglesRaw);
	for (int i = 0; i < trianglesRaw.size(); i++) {
		if (
			trianglesRaw[i][0] > 0 &&
			trianglesRaw[i][2] > 0 &&
			trianglesRaw[i][4] > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][0],trianglesRaw[i][1])].x > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][2],trianglesRaw[i][3])].x > 0 &&
			nextPosition_[Point2f(trianglesRaw[i][4],trianglesRaw[i][5])].x > 0
			)
			triangles.push_back(trianglesRaw[i]);
	}

	vector<float> timeToCollides;
	vector<float> shapeRatios;

	Mat riskMapTest(config_.frameHeight, config_.frameWidth, CV_8UC1);

	vector<vector<Point>> trianglesBefore, trianglesAfter;
	for (int i = 0; i < triangles.size(); i++) {
		Vec6f t = triangles[i];
		vector<Point> pointsBefore;
		pointsBefore.push_back(Point(t[0], t[1]));
		pointsBefore.push_back(Point(t[2], t[3]));
		pointsBefore.push_back(Point(t[4], t[5]));

		vector<Point> pointsAfter;
		pointsAfter.push_back(nextPosition_[pointsBefore[0]]);
		pointsAfter.push_back(nextPosition_[pointsBefore[1]]);
		pointsAfter.push_back(nextPosition_[pointsBefore[2]]);
		
		trianglesBefore.push_back(pointsBefore);
		trianglesAfter.push_back(pointsAfter);

		// fix false alarm on lateral motions
		float shapeRatio = getShapeRatio(pointsBefore, pointsAfter);
		shapeRatios.push_back(shapeRatio);

		float areaBefore = contourArea(pointsBefore);
		float areaAfter = contourArea(pointsAfter);

		float lenBefore = sqrt(areaBefore);
		float lenAfter = sqrt(areaAfter);

		float ttc = lenBefore / (lenAfter - lenBefore);
		//float ttc = sqrt(areaAfter * areaBefore) / (areaAfter - areaBefore);
		timeToCollides.push_back(ttc);

		Scalar color = Scalar(ttc >= 0 ? (255 - ttc * shapeRatio) : 0);
		fillConvexPoly(riskMap_, pointsAfter, color);

		fillConvexPoly(riskMapTest, pointsAfter, Scalar(ttc >= 0 ? (255 - ttc) : 0));
	}

#ifdef DEBUG
	// breakdown
	Mat trackedPoints = previousFrameRaw_.clone();
	Mat optFlow = previousFrameRaw_.clone();
	Mat delaunay = previousFrameRaw_.clone();
	Mat delaunayCompare = previousFrameRaw_.clone();
	for (int i = 0; i < previousTrackedPoints_.size(); i++) {
		circle(trackedPoints, previousTrackedPoints_[i], 4, Scalar(0, 255, 255), -1);

		circle(optFlow, currentTrackedPoints_[i], 4, Scalar(0, 0, 255), -1);
		line(optFlow, previousTrackedPoints_[i], currentTrackedPoints_[i], Scalar(0, 255, 255), 2);
	}
	for (int i = 0; i < triangles.size(); i++) {
		Utils::drawTriangle(delaunay, trianglesBefore[i], Scalar(255, 255, 0), 1);

		Utils::drawTriangle(delaunayCompare, trianglesBefore[i], Scalar(255, 20, 0), 1);
		line(delaunayCompare, trianglesBefore[i][0], trianglesAfter[i][0], Scalar(0, 120, 255), 1);
		line(delaunayCompare, trianglesBefore[i][1], trianglesAfter[i][1], Scalar(0, 120, 255), 1);
		line(delaunayCompare, trianglesBefore[i][2], trianglesAfter[i][2], Scalar(0, 120, 255), 1);
		Utils::drawTriangle(delaunayCompare, trianglesAfter[i], Scalar(255, 255, 0), 1);
		
		int areaBefore = (int)contourArea(trianglesBefore[i]);
		int areaAfter = (int)contourArea(trianglesAfter[i]);
		float ttc = sqrt(areaAfter * areaBefore) / (areaAfter - areaBefore);
		float ttc2 = sqrt(areaBefore) / (sqrt(areaAfter) - sqrt(areaBefore));
		float ttc3 = ttc2 * shapeRatios[i];

		if (ttc < TTC_LOW && ttc >= 0) {
			Utils::drawTriangle(delaunayCompare, trianglesAfter[i], Scalar(0, 0, 255), 2);
			Point center = Utils::getCenter(trianglesAfter[i]);

			putText(delaunayCompare, to_string(ttc) + " " + to_string(ttc2) + " " + to_string(ttc3),
				center, CV_FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 255, 255), 2);
			putText(delaunayCompare, to_string(areaAfter) + "x" + to_string(areaBefore),
				center + Point(0, 10), CV_FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 255, 255), 1);
			putText(delaunayCompare, to_string(shapeRatios[i]),
				center + Point(0, 20), CV_FONT_HERSHEY_PLAIN, 0.7, Scalar(0, 255, 255), 1);
		}
	}

	imshow("Tracked Points", trackedPoints);
	imshow("rt", riskMapTest);
	imshow("Optical Flow", optFlow);
	imshow("Delaunay Triangulation", delaunay);
	imshow("Delaunay Before/After", delaunayCompare);
	imshow("r", riskMap_);
	//imshow("shape", shape);
#endif 
}



map<Point2f, Point2f, Utils::cmpPoint2f> HitAlert::getTrackedPoints()
{
	return nextPosition_;
}



Mat HitAlert::getPreviousFrame()
{
	return previousFrame_;
}



Mat HitAlert::getCurrentFrame()
{
	return currentFrame_;
}



Mat HitAlert::getRiskMap()
{
	return riskMap_;
}



Size2f HitAlert::getBoundingSize(vector<Point> points)
{
	int minX = points[0].x;
	int minY = points[0].y;
	int maxX = points[0].x;
	int maxY = points[0].y;

	for (int i = 0; i < points.size(); i++) {
		if (points[i].x < minX) 
			minX = points[i].x;
		if (points[i].y < minY)
			minY = points[i].y;
		if (points[i].x > maxX)
			minX = points[i].x;
		if (points[i].y > maxY)
			minY = points[i].y;
	}

	return Size2f(maxX - minX, maxY - minY);
}



inline float HitAlert::getShapeRatio(vector<Point> pointsBefore, vector<Point> pointsAfter) {
    //Size2f boundSizeBefore = getBoundingSize(pointsBefore);
    //Size2f boundSizeAfter = getBoundingSize(pointsAfter);

    //float heightRatio = boundSizeAfter.height / boundSizeBefore.height;
    //float widthRatio = boundSizeAfter.width / boundSizeBefore.width;
    //float shapeRatio = heightRatio / widthRatio;
    //if (shapeRatio < 1)
    //	shapeRatio = 1 / shapeRatio;
    //
    //// try...
    //if (shapeRatio > 1.01) {
    //	shapeRatio = pow(shapeRatio, 10);
    //}

    //vector<float> distRatios;
    float distRatioMean = 0;

    Point frameCenter(config_.frameWidth / 2, config_.frameHeight / 2);
    frameCenter.x += optFlowMean_.x;
    frameCenter.y += optFlowMean_.y;

    float minDistRatio = 1000, maxDistRatio = 0;

    for (int i = 0; i < pointsBefore.size(); i++) {
        // TODO check?
        float distRatio = norm((pointsBefore[i] - frameCenter)) / norm((pointsAfter[i] - frameCenter));
        //distRatios.push_back(distRatio);
        //distRatioMean += distRatio;

        if (distRatio > maxDistRatio)
            maxDistRatio = distRatio;
        if (distRatio < minDistRatio)
            minDistRatio = distRatio;
    }

    //distRatioMean /= pointsBefore.size();
    float shapeRatio = maxDistRatio - minDistRatio;
    shapeRatio = shapeRatio > DIST_RATIO_RANGE_THRESH ?
                 1 + (shapeRatio - DIST_RATIO_RANGE_THRESH) * DIST_RATIO_COEFF : 1;
    //shapeRatio = shapeRatio * 100;

    return shapeRatio;
}