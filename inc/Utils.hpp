#pragma once

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class Utils {
public:
	static void drawFps(Mat img);
};