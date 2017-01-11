// -------------------- debugs --------------------
//#define DEBUG
//#define COMPARE


// -------------------- paths --------------------
#define HITALERT_BASE_PATH "../../../hitalert/"
#define HITALERT_RUNTIME_PATH "" //HITALERT_BASE_PATH

#define SOUND_C_HI HITALERT_RUNTIME_PATH "res/MidHigh.wav"
#define SOUND_R_HI HITALERT_RUNTIME_PATH "res/RightHigh.wav"
#define SOUND_L_HI HITALERT_RUNTIME_PATH "res/LeftHigh.wav"
#define SOUND_C_LOW HITALERT_RUNTIME_PATH "res/MidLow.wav"
#define SOUND_R_LOW HITALERT_RUNTIME_PATH "res/RightLow.wav"
#define SOUND_L_LOW HITALERT_RUNTIME_PATH "res/LeftLow.wav"


// -------------------- input --------------------
#define USE_WEBCAM
#define CAP_CAM_NO 0
#define CAP_VID_PATH HITALERT_BASE_PATH "test_data/"
#define CAP_VID_NAME "07.mp4"


// -------------------- output --------------------
//#define OUTPUT_VIDEO
#define OUTPUT_VIDEO_PATH HITALERT_BASE_PATH "output/out.mp4"

//#define ALERT_LEFT_RIGHT 
#define ALERT_HIGH_ONLY


// -------------------- image config --------------------
#define PROCESS_WIDTH 480//720
#define PROCESS_HEIGHT 360//480
#define FRAMES_SKIP 0
#define FRAME_PADDING 100
#define ROI_Y_OFFSET 40


#define ROI_WIDTH_REL 0.36
// calculated from human dimension of 60 cm in width, 50cm distance with 60 deg FOV camera


// -------------------- hitalert parameters --------------------
#define TRACK_CORNERS 150
#define TRACK_QUALITY 0.05
#define TRACK_MIN_DIST 20

#define TTC_LOW 80
#define TTC_MID 120

#define GS_BLUR_SIZE 25
#define GS_BLUR_SIGMA_REL 0.7
#define EXP_BLUR_RATIO 0.8

#define DIST_RATIO_RANGE_THRESH 0.03
#define DIST_RATIO_COEFF 100



//////////////////// derived configs ////////////////////

#ifdef USE_WEBCAM
#define CAP_SRC 0
#else
#define CAP_SRC CAP_VID_PATH CAP_VID_NAME
#endif
