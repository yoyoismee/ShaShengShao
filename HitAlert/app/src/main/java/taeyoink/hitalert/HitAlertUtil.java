package taeyoink.hitalert;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfFloat6;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Subdiv2D;

import java.util.Map;
import java.util.Vector;

import static java.lang.Math.sqrt;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.fillConvexPoly;
import static org.opencv.imgproc.Imgproc.goodFeaturesToTrack;
import static org.opencv.imgproc.Imgproc.resize;
import static org.opencv.video.Video.calcOpticalFlowPyrLK;

public class HitAlertUtil {
    private  int frameWidth;
    private  int frameHeight;
    private  int trackMaxCorners;
    private  float trackQuality;
    private  int trackMinDistance;
    private  MatOfPoint previousTrackedPointss;
    private  MatOfPoint2f previousTrackedPoints,currentTrackedPoints;
    private Map<Point,Point> nextPosition;
    private Point optFlowMean = new Point(0, 0);


    private  Mat currentFrame,currentFrameRaw,previousFrameRaw,previousFrame,riskMap;
    //Intent intent = new Intent(android.provider.MediaStore.ACTION_IMAGE_CAPTURE);
    public HitAlertUtil(int sframeWidth,
             int sframeHeight,
             int strackMaxCorners,
             float strackQuality,
             int strackMinDistance){
        frameWidth=sframeWidth;
        frameHeight=sframeHeight;
        trackMaxCorners=strackMaxCorners;
        trackQuality=strackQuality;
        trackMinDistance=strackMinDistance;
    }

    //private static Camera cam ; //not working
    public static final int ALERT_High_Left = 2;
    public static final int ALERT_High_Mid = 20;
    public static final int ALERT_High_Right = 200;
    public static final int ALERT_Low_Left = 1;
    public static final int ALERT_Low_Mid = 10;
    public static final int ALERT_Low_Right = 100;
    public static void Alert (int info){
        // one parameter combind by + i.e. Alert(ALERT_High_Mid + ALERT_Low_Left)
    }
        /*TODO*/

    public static void inni(){
        //cam = Camera.open(); //not working
    }
    public static void stop(){
        //cam.release(); //not working
    }
    public static void CapMat(Size s,Mat out){
         out = new Mat(s, CV_8UC1);
       // cam.takePicture(Camera.ShutterCallback, Camera.PictureCallback, Camera.PictureCallback, Camera.PictureCallback);
        //startActivityForResult(intent,0)
        /*TODO*/ //not working
    }

    public  void setCurrentFrame(Mat newFrame){
        resize(newFrame, currentFrameRaw, new Size(frameWidth, frameHeight));

        Mat newFrameGray=null;
        cvtColor(newFrame, newFrameGray, COLOR_BGR2GRAY);
        resize(newFrameGray, currentFrame, new Size(frameWidth, frameHeight));
    }

    public  void pushFrame()
    {
        previousFrame = currentFrame.clone();
        previousFrameRaw = currentFrameRaw.clone();
    }

    public Map<Point, Point> getTrackedPoints()
    {
        return nextPosition;
    }



    public Mat getPreviousFrame()
    {
        return previousFrame;
    }



    public Mat getCurrentFrame()
    {
        return currentFrame;
    }



    public Mat getRiskMap()
    {
        return riskMap;
    }



    public Size getBoundingSize(Vector<Point> points)
    {
        double minX = points.get(0).x;
        double minY = points.get(0).y;
        double maxX = points.get(0).x;
        double maxY = points.get(0).y;

        for (int i = 0; i < points.size(); i++) {
            if (points.get(i).x < minX)
                minX = points.get(i).x;
            if (points.get(i).y < minY)
                minY = points.get(i).y;
            if (points.get(i).x > maxX)
                minX = points.get(i).x;
            if (points.get(i).y > maxY)
                minY = points.get(i).y;
        }

        return new Size(maxX - minX, maxY - minY);
    }



    public float getShapeRatio(Vector<Point> pointsBefore, Vector<Point> pointsAfter) {
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

        Point frameCenter(frameWidth / 2, frameHeight / 2);
        frameCenter.x += optFlowMean.x;
        frameCenter.y += optFlowMean.y;

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
    public int calculateRiskmap (){
            //riskMap = 0;  /*TODO*/
            //nextPosition.clear();
            goodFeaturesToTrack(
                    previousFrame, previousTrackedPointss,
                    trackMaxCorners,
                    trackQuality,
                    trackMinDistance);
            if (previousTrackedPointss.size().area() == 0) { //fixme
                return 1;
            }

            MatOfByte statuss=null;
            MatOfFloat err=null;
            MatOfPoint2f previousTrackedPoints = new MatOfPoint2f(previousTrackedPoints.toArray());
            calcOpticalFlowPyrLK(
                    previousFrame, currentFrame,
                    previousTrackedPoints, currentTrackedPoints,
                    statuss, err);
            byte[] status = statuss.toArray();
            // delaunay
            int validOptFlowCount = 0;
            Subdiv2D subdiv = new Subdiv2D(new Rect(0, 0,frameWidth,frameHeight));
        //fixme
        for (int i = 0; i < (int)previousTrackedPoints.size().area(); i++) {
            if ((status[i] & 2 ^ i) != 0) { //fixme
                subdiv.insert(previousTrackedPoints.toArray()[i]);
                nextPosition.put(previousTrackedPoints.toArray()[i], currentTrackedPoints.toArray()[i]);
                optFlowMean.x += (currentTrackedPoints.toArray()[i].x - previousTrackedPoints.toArray()[i].x);
                optFlowMean.y += (currentTrackedPoints.toArray()[i].y - previousTrackedPoints.toArray()[i].y);
                validOptFlowCount++;
            }
        }
            optFlowMean.x /= validOptFlowCount;
            optFlowMean.y /= validOptFlowCount;
            MatOfFloat6 trianglesRaw,triangles;
            subdiv.getTriangleList(trianglesRaw); //fixme
            for (int i = 0; i < trianglesRaw.rows(); i++) { //fixme?
                if (
                        trianglesRaw.get(i, 0)[0] > 0 &&
                                trianglesRaw.get(i,0)[2] > 0 &&
                                trianglesRaw.get(i,0)[4] > 0 &&
                                nextPosition.get(new Point(trianglesRaw.get(i,0)[0],trianglesRaw.get(i,0)[1])).x > 0 &&
                                nextPosition.get(new Point(trianglesRaw.get(i,0)[2],trianglesRaw.get(i,0)[3])).x > 0 &&
                                nextPosition.get(new Point(trianglesRaw.get(i,0)[4],trianglesRaw.get(i,0)[5])).x > 0
                        )
                    triangles.push_back(trianglesRaw.row(i));

            }

            Vector<Float> timeToCollides;
            Vector<Float> shapeRatios;

            Mat riskMapTest = new Mat(frameHeight,frameWidth, CV_8UC1);

            Vector<Vector<Point>> trianglesBefore=new Vector<Vector<Point>>(), trianglesAfter=new Vector<Vector<Point>>();
            for (int i = 0; i < triangles.rows(); i++) {
                MatOfFloat6 t = (MatOfFloat6) triangles.row(i);
                Vector<Point> pointsBefore= new Vector<Point>();
                pointsBefore.add(pointsBefore.size(),(new Point(t.get(0,0)[0], t.get(0,0)[1])));
                pointsBefore.add(pointsBefore.size(),new Point(t.get(0,0)[0], t.get(0,0)[1]));
                pointsBefore.add(pointsBefore.size(),new Point(t.get(0,0)[2], t.get(0,0)[3]));
                pointsBefore.add(pointsBefore.size(),new Point(t.get(0,0)[4], t.get(0,0)[5]));

                Vector<Point> pointsAfter = new Vector<Point>();
                pointsAfter.add(pointsAfter.size(),nextPosition.get(pointsBefore.get(0)));
                pointsAfter.add(pointsAfter.size(),nextPosition.get(pointsBefore.get(1)));
                pointsAfter.add(pointsAfter.size(),nextPosition.get(pointsBefore.get(2)));

                trianglesBefore.add(trianglesBefore.size(),pointsBefore);
                trianglesAfter.add(trianglesAfter.size(),pointsAfter);

                // fix false alarm on lateral motions
                double shapeRatio = getShapeRatio(pointsBefore, pointsAfter);
                shapeRatios.push_back(shapeRatio);

                double areaBefore = contourArea(pointsBefore);
                double areaAfter = contourArea(pointsAfter);

                double lenBefore = sqrt(areaBefore);
                double lenAfter = sqrt(areaAfter);

                double ttc = lenBefore / (lenAfter - lenBefore);
                //double ttc = sqrt(areaAfter * areaBefore) / (areaAfter - areaBefore);
                timeToCollides.push_back(ttc);

                Scalar color = Scalar(ttc >= 0 ? (255 - ttc * shapeRatio) : 0);
                fillConvexPoly(riskMap, pointsAfter, color);

                fillConvexPoly(riskMapTest, pointsAfter, Scalar(ttc >= 0 ? (255 - ttc) : 0));

    }

}
