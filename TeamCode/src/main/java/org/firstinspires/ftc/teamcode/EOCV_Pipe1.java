package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCV_Pipe1 extends OpenCvPipeline {

    int Final_Result = 0;

    Mat mat = new Mat();

    Rect leftROI = new Rect(new Point(100, 100), new Point(300, 300));
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
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBound = new Scalar(1, 60, 70);
        Scalar upperBound = new Scalar(15, 100, 100);
        Core.inRange(mat, lowerBound, upperBound, mat);

        //DIVIDE
        leftMat = mat.submat(leftROI);
        rightMat = mat.submat(rightROI);
        centerMat = mat.submat(centerROI);

        //AVERAGE
        double leftVal = Math.round(Core.mean(leftMat).val[2] / 255);
        double rightVal = Math.round(Core.mean(rightMat).val[2] / 255);
        double centerVal = Math.round(Core.mean(centerMat).val[2] / 255);

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

        //RELEASE
        leftMat.release();
        rightMat.release();
        centerMat.release();
        mat.release();

        //DRAW RECTS
        Imgproc.rectangle(input, new Point(100, 100), new Point(300, 300), new Scalar(0, 255, 0), 5);

        telemetry.addData("RESULT", Final_Result);
        telemetry.addData("LEFT", leftVal);
        telemetry.addData("RIGHT", rightVal);
        telemetry.addData("CENTER", centerVal);
        telemetry.update();

        return input;
    }

    public int getFinal_Result() {
        return Final_Result;
    }

    public void stop() {
        cam.stopStreaming();
    }
}
