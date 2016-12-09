#define PROCESS_WIDTH 640//480//720//640 // 400
#define PROCESS_HEIGHT 480//360//270//480//480// 360 // 225
#define FRAMES_SKIP 0

#define ROI_COUNT 3
#define ROI_WIDTH_REL 0.35
// calculated from human dimension of 60 cm in width, 50cm distance with 60 deg FOV camera

#define TRACK_CORNERS 150
#define TRACK_QUALITY 0.05
#define TRACK_MIN_DIST 20

#define TTC_LOW 80
#define TTC_MID 110

#define GS_BLUR_SIZE 25
#define GS_BLUR_SIGMA_REL 0.7
#define EXP_BLUR_RATIO 0.9

#define DIST_RATIO_RANGE_THRESH 0.03
#define DIST_RATIO_COEFF 100

//#define USE_WEBCAM
#define CAP_CAM_NO 0
#define CAP_VID_PATH "../../../hitalert/test_data/"
#define CAP_VID_NAME "01.mp4"

#define DEBUG
#define COMPARE