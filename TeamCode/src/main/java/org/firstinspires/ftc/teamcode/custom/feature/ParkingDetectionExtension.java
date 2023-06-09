package org.firstinspires.ftc.teamcode.custom.feature;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 * Creates a camera and OpenCV pipeline which scans for AprilTags each loop and saves the last detected position.
 */
public class ParkingDetectionExtension {

    public enum ParkingPosition {

        LEFT,
        CENTER,
        RIGHT;

        public static final ParkingPosition[] POSITIONS = values();

    }

    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private final Telemetry telemetry;
    private final AprilTagCameraPipeline pipeline;
    private final OpenCvCamera camera;

    private ParkingPosition parkingPosition = null;

    public ParkingDetectionExtension(PowerPlayRobot robot, HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, robot.WEBCAM_NAME), cameraMonitorViewId);
        pipeline = new AprilTagCameraPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("[!] Error caught in the camera pipeline!");
                telemetry.addLine("[!] Error code: " + errorCode);
            }
        });

        telemetry.setMsTransmissionInterval(50);
        camera.closeCameraDevice();
    }

    public void stop() {

        camera.closeCameraDeviceAsync(()-> {

        });

    }

    public void init_loop() {
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if (detections != null) {

            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    pipeline.setDecimation(DECIMATION_LOW);
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;
                // If the target is within 1 meter, turn on high decimation to increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    pipeline.setDecimation(DECIMATION_HIGH);

                for (AprilTagDetection detection : detections) {
                    if (detection.id >= 3) return; // happened for some reason
                    parkingPosition = ParkingPosition.POSITIONS[detection.id];
                }

            }

        }

        // Camera Feedback
        if (parkingPosition == null)
            telemetry.addLine("[!] Parking is NOT READY! Robot has not detected an AprilTag...");
        else
            telemetry.addLine("[✓] Parking is READY! Detected " + parkingPosition.name() + " (" + (parkingPosition.ordinal() + 1) + ")");
        telemetry.update();

    }

    public ParkingPosition getParkingPosition() {
        return parkingPosition;
    }

    @Config
    public static class AprilTagCameraPipeline extends OpenCvPipeline {

        // The example pipeline provided by AprilTags does some things for "fun" such as drawing on shapes on the image output.
        // While this can be useful for testing, it is not helpful during competition and requires resources.
        // I don't know if the resources used by this are significant, but either way, this option lets you change whether line drawing is done.
        public static boolean OPTIMIZE_RESOURCES = true;

        private long nativeAprilTagPointer;
        private final Mat grey = new Mat();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        Mat cameraMatrix;

        Scalar blue = new Scalar(7,197,235,255);
        Scalar red = new Scalar(255,0,0,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);

        double fx;
        double fy;
        double cx;
        double cy;

        // UNITS ARE METERS
        double tagsize;
        double tagsizeX;
        double tagsizeY;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        public AprilTagCameraPipeline(double tagsize, double fx, double fy, double cx, double cy) {
            this.tagsize = tagsize;
            this.tagsizeX = tagsize;
            this.tagsizeY = tagsize;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            constructMatrix();

            // Allocate a native context object. See the corresponding deletion in the finalizer
            nativeAprilTagPointer = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        protected void finalize() {
            // Might be null if createAprilTagDetector() threw an exception
            if(nativeAprilTagPointer != 0) {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPointer);
                nativeAprilTagPointer = 0;
            }
            else {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync) {
                if(needToSetDecimation) {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPointer, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPointer, grey, tagsize, fx, fy, cx, cy);

            synchronized (detectionsUpdateSync) {
                detectionsUpdate = detections;
            }

            if (!OPTIMIZE_RESOURCES) {
                // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
                // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
                for(AprilTagDetection detection : detections) {
                    Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                    drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                    draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
                }
            }

            return input;
        }

        public void setDecimation(float decimation) {
            synchronized (decimationSync) {
                this.decimation = decimation;
                needToSetDecimation = true;
            }
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate() {
            synchronized (detectionsUpdateSync) {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void constructMatrix() {

            //     Construct the camera matrix.
            //
            //      --         --
            //     | fx   0   cx |
            //     | 0    fy  cy |
            //     | 0    0   1  |
            //      --         --
            //

            cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

            cameraMatrix.put(0,0, fx);
            cameraMatrix.put(0,1,0);
            cameraMatrix.put(0,2, cx);

            cameraMatrix.put(1,0,0);
            cameraMatrix.put(1,1,fy);
            cameraMatrix.put(1,2,cy);

            cameraMatrix.put(2, 0, 0);
            cameraMatrix.put(2,1,0);
            cameraMatrix.put(2,2,1);
        }

        /**
         * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
         *
         * @param buf the RGB buffer on which to draw the marker
         * @param length the length of each of the marker 'poles'
         * @param rvec the rotation vector of the detection
         * @param tvec the translation vector of the detection
         * @param cameraMatrix the camera matrix used when finding the detection
         */
        void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(0,0,0),
                    new Point3(length,0,0),
                    new Point3(0,length,0),
                    new Point3(0,0,-length)
            );

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Draw the marker!
            Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
            Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

            Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
        }

        void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
            //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

            // The points in 3D space we wish to project onto the 2D image plane.
            // The origin of the coordinate space is assumed to be in the center of the detection.
            MatOfPoint3f axis = new MatOfPoint3f(
                    new Point3(-tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2, tagHeight/2,0),
                    new Point3( tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2,-tagHeight/2,0),
                    new Point3(-tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2, tagHeight/2,-length),
                    new Point3( tagWidth/2,-tagHeight/2,-length),
                    new Point3(-tagWidth/2,-tagHeight/2,-length));

            // Project those points
            MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
            Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
            Point[] projectedPoints = matProjectedPoints.toArray();

            // Pillars
            for(int i = 0; i < 4; i++) {
                Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
            }

            // Base lines
            //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
            //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
            //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
            //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

            // Top lines
            Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
            Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
            Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
            Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
        }

        /**
         * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
         * original size of the tag.
         *
         * @param points the points which form the trapezoid
         * @param cameraMatrix the camera intrinsics matrix
         * @param tagsizeX the original width of the tag
         * @param tagsizeY the original height of the tag
         * @return the 6DOF pose of the camera relative to the tag
         */
        Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY) {
            // The actual 2d points of the tag detected in the image
            MatOfPoint2f points2d = new MatOfPoint2f(points);

            // The 3d points of the tag in an 'ideal projection'
            Point3[] arrayPoints3d = new Point3[4];
            arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
            arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
            arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
            MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

            // Using this information, actually solve for pose
            Pose pose = new Pose();
            Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

            return pose;
        }

        /*
         * A simple container to hold both rotation and translation
         * vectors, which together form a 6DOF pose.
         */
        static class Pose {
            Mat rvec;
            Mat tvec;

            public Pose()
            {
                rvec = new Mat();
                tvec = new Mat();
            }

            public Pose(Mat rvec, Mat tvec)
            {
                this.rvec = rvec;
                this.tvec = tvec;
            }
        }

    }

}
