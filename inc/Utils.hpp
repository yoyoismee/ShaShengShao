#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

class Utils {
public:
	static void drawFps(Mat img);
	static void drawTriangle(Mat img, vector<Point> points, Scalar color, int thickness = 1);
	static Point getCenter(vector<Point> points);

	struct cmpPoint2f {
		bool operator()(const Point2f& a, const Point2f& b) const {
			return a.x != b.x ? a.x < b.x : a.y < b.y;
		}
	};

	struct cmpPoint {
		bool operator()(const Point& a, const Point& b) const {
			return a.x != b.x ? a.x < b.x : a.y < b.y;
		}
	};
};