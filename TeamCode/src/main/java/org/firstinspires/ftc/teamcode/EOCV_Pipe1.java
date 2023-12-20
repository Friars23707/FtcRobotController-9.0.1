package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCV_Pipe1 extends OpenCvPipeline {

    int Final_Result = 0;

    Mat mat = new Mat();
    Rect leftROI = new Rect(new Point(100, 100), new Point(300, 100));
    Mat leftMat;
    Rect rightROI = new Rect(new Point(100, 100), new Point(300, 100));
    Mat rightMat;
    Rect centerROI = new Rect(new Point(100, 100), new Point(300, 100));
    Mat centerMat;

    private Telemetry telemetry;
    private OpenCvCamera cam;

    public EOCV_Pipe1(HardwareMap hardwareMap, Telemetry t) {
        telemetry = t;
        int cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraViewId);
        cam.setPipeline(this);
        cam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
    }
    @Override
    public Mat processFrame(Mat input) {

        //THRESHOLD
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBound = new Scalar(0, 70, 60);
        Scalar upperBound = new Scalar(15.0 / 2, 255, 255);
        Core.inRange(mat, lowerBound, upperBound, mat);

        //DIVIDE
        leftMat = mat.submat(leftROI);
        rightMat = mat.submat(rightROI);
        centerMat = mat.submat(centerROI);

        //AVERAGE
        double leftVal = Math.round(Core.mean(leftMat).val[2] / 255);
        double rightVal = Math.round(Core.mean(rightMat).val[2] / 255);
        double centerVal = Math.round(Core.mean(centerMat).val[2] / 255);

        //RELEASE
        leftMat.release();
        rightMat.release();
        centerMat.release();
        mat.release();

        //COMPARE
        if (leftVal > rightVal && leftVal > centerVal) {
            Final_Result = 1;
        } else if (rightVal > centerVal && rightVal > leftVal) {
            Final_Result = 3;
        } else if (centerVal > leftVal && centerVal > rightVal) {
            Final_Result = 2;
        } else {
            Final_Result = 4;
        }

        return null;
    }

    public int getFinal_Result() {
        return Final_Result;
    }

    public void stop() {
        cam.stopStreaming();
    }
}
