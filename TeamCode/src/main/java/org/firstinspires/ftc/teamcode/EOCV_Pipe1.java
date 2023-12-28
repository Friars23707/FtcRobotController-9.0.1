package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Finalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCV_Pipe1 extends OpenCvPipeline {

    int Final_Red = 0;
    int Final_Blue = 0;

    Mat redMat = new Mat();
    Mat blueMat = new Mat();

    Rect leftROI = new Rect(new Point(100, 200), new Point(350, 600));
    Mat leftRedMat;
    Mat leftBlueMat;
    Rect rightROI = new Rect(new Point(1100, 200), new Point(800, 600));
    Mat rightRedMat;
    Mat rightBlueMat;
    Rect centerROI = new Rect(new Point(425, 200), new Point(700, 600));
    Mat centerRedMat;
    Mat centerBlueMat;

    private Telemetry telemetry;
    private OpenCvCamera cam;

    public Scalar hsv(double h, double s, double v) {
        return new Scalar(h/2.0, s/100.0*255, v/100.0*255);
    }

    public EOCV_Pipe1(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        int cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraViewId);
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
    @Override
    public Mat processFrame(Mat input) {

        //THRESHOLD
        //Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(input, redMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, blueMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerRedBound = hsv(1.0, 0.0, 0.0);
        Scalar upperRedBound = hsv(15.0, 100.0, 100.0);
        Core.inRange(redMat, lowerRedBound, upperRedBound, redMat);

        Scalar lowerBlueBound = hsv(1.0, 0.0, 0.0);
        Scalar upperBlueBound = hsv(15.0, 100.0, 100.0);
        Core.inRange(blueMat, lowerRedBound, upperRedBound, blueMat);



        //DIVIDE
        leftRedMat = redMat.submat(leftROI);
        rightRedMat = redMat.submat(rightROI);
        centerRedMat = redMat.submat(centerROI);

        leftBlueMat = blueMat.submat(leftROI);
        rightBlueMat = blueMat.submat(rightROI);
        centerBlueMat = blueMat.submat(centerROI);

        //AVERAGE
        //OLD: Math.round(Core.mean(rightMat).val[2] / 255);
        double redLeftVal = Core.countNonZero(leftRedMat);
        double redRightVal = Core.countNonZero(rightRedMat);
        double redCenterVal = Core.countNonZero(centerRedMat);

        double blueLeftVal = Core.countNonZero(leftBlueMat);
        double blueRightVal = Core.countNonZero(rightBlueMat);
        double blueCenterVal = Core.countNonZero(centerBlueMat);

        //COMPARE
        if (redLeftVal > redRightVal && redLeftVal > redCenterVal) {
            Final_Red = 1;
        } else if (redRightVal > redCenterVal && redRightVal > redLeftVal) {
            Final_Red = 3;
        } else if (redCenterVal > redLeftVal && redCenterVal > redRightVal) {
            Final_Red = 2;
        } else {
            Final_Red = 4;
        }

        if (blueLeftVal > blueRightVal && blueLeftVal > blueCenterVal) {
            Final_Blue = 1;
        } else if (blueRightVal > blueCenterVal && blueRightVal > blueLeftVal) {
            Final_Blue = 3;
        } else if (blueCenterVal > blueLeftVal && blueCenterVal > blueRightVal) {
            Final_Blue = 2;
        } else {
            Final_Blue = 4;
        }

        //RELEASE
        leftRedMat.release();
        rightRedMat.release();
        centerRedMat.release();
        redMat.release();
        leftBlueMat.release();
        rightBlueMat.release();
        centerBlueMat.release();
        blueMat.release();

        //DRAW RECTS
        Imgproc.rectangle(input, new Point(100, 200), new Point(350, 600), new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(input, new Point(1100, 200), new Point(800, 600), new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(input, new Point(425, 200), new Point(700, 600), new Scalar(0, 0, 255), 5);

        telemetry.addData("RED", Final_Red);
        telemetry.addData("BLUE", Final_Blue);
        telemetry.update();

        return input;
    }

    public int getFinal_Red() {
        return Final_Red;
    }

    public int getFinal_Blue() {
        return Final_Blue;
    }

    public void stop() {
        cam.stopStreaming();
    }
}
