#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Utils.hpp"
#include <ctime>
#include <string>
#include <sstream>

using namespace cv;
using namespace std;

namespace patch
{
	template < typename T > std::string to_string( const T& n )
	{
		std::ostringstream stm ;
		stm << n ;
		return stm.str() ;
	}
}


void Utils::drawFps(Mat img)
{
	static clock_t lastFrameTime;
	clock_t thisFrameTime = clock();
	float dt = (float)(thisFrameTime - lastFrameTime) / CLOCKS_PER_SEC;
	lastFrameTime = thisFrameTime;
	putText(img, patch::to_string(1 / dt), Point(20, 40),
		CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 255));
}



void Utils::drawTriangle(Mat img, vector<Point> points, Scalar color, int thickness)
{
	line(img, points[0], points[1], color, thickness);
	line(img, points[1], points[2], color, thickness);
	line(img, points[2], points[0], color, thickness);
}



Point Utils::getCenter(vector<Point> points)
{
	Point c;
	for (int i = 0; i < points.size(); i++) {
		c += points[i];
	}
	c.x /= points.size();
	c.y /= points.size();
	return c;
}