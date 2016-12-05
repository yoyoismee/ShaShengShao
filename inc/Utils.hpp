#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class Utils {
public:
	static void drawFps(Mat img);

	struct cmpPoint2f {
		bool operator()(const Point2f& a, const Point2f& b) const {
			return a.x != b.x ? a.x < b.x : a.y < b.y;
		}
	};
};