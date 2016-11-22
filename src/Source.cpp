#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Utils.hpp"
#include <ctime>

using namespace cv;
using namespace std;

void Utils::drawFps(Mat img) {
	static clock_t lastFrameTime;
	clock_t thisFrameTime = clock();
	float dt = (float)(thisFrameTime - lastFrameTime) / CLOCKS_PER_SEC;
	lastFrameTime = thisFrameTime;
	putText(img, to_string(1 / dt), Point(20, 40),
		CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255));
}