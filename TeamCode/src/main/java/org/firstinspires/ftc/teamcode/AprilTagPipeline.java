package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="AprilTagPipeline", group="Linear OpMode")
public class AprilTagPipeline extends LinearOpMode {
    public VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;

    public double yawError = 0;
    public double bearningError = 0;
    public double rangeError = 0;

    public Telemetry telem;

    public AprilTagPipeline(Telemetry tm) {
        telem = tm;
    }

    public  void initVision(HardwareMap hwM) {
        // Initialize the Apriltag Detection process
        initAprilTag(hwM);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    public void orient(SampleMecanumDrive drive, int tagID, HardwareMap hwM) {
        telem.addData("PLSPLS", yawError);
        telem.addData("PLSPLS2", bearningError);
        telem.update();
        getErrors(tagID, hwM);
        telem.addData("erros", "found");
        telem.addData("range", desiredTag.ftcPose.range);
        telem.addData("bearing", bearningError);
        telem.addData("yaw", desiredTag.ftcPose.yaw);
        telem.update();
        sleep(5000);
        if (bearningError == 0.0 || yawError == 0.0 || rangeError == 0.0) { return; }
        TrajectorySequence fix3 = drive.trajectorySequenceBuilder(new Pose2d())
                //.strafeRight(bearningError) //yawError
                //.back(rangeError-12) //rangeError-12
                .turn(Math.toRadians(yawError * 1.1)) //Math.toRadians(bearningError)
                .build();
        drive.followTrajectorySequence(fix3);
        telem.addData("RUNNING RR", "YESSSSS");
        telem.update();
        sleep(1000);
        getErrors(tagID, hwM);
        if (bearningError == 0.0 || yawError == 0.0 || rangeError == 0.0) { return; }
        TrajectorySequence fix4 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(-bearningError + 0.5) //yawError
                .back(rangeError-12) //rangeError-12
                //.turn(Math.toRadians(yawError * 1.05)) //Math.toRadians(bearningError)
                .build();
        drive.followTrajectorySequence(fix4);
        sleep(1000);
        getErrors(tagID, hwM);
        if (bearningError == 0.0 || yawError == 0.0 || rangeError == 0.0) { return; }
        telem.addData("YAW", yawError);
        telem.update();
        TrajectorySequence fix5 = drive.trajectorySequenceBuilder(new Pose2d())
                //.strafeRight(bearningError) //yawError
                //.back(rangeError-12) //rangeError-12
                .turn(Math.toRadians(yawError * 1.1)) //Math.toRadians(bearningError)
                .build();
        drive.followTrajectorySequence(fix5);
        telem.addData("BRUHHHH", "NO");
        telem.update();
        sleep(1000);
        return;
    }

    public void getErrors(int tagID, HardwareMap hwM) {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        while (!isStopRequested() && !targetFound) {
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection ele: currentDetections) {
                telem.addData("added", ele);
            }
            telem.addData("size", currentDetections.size());
            telem.addData("TEST", "e");
            telem.update();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (isStopRequested()) {
                    break;
                }
                telem.addData("id-f", detection.id);
                if (detection.metadata != null) {
                    // Yes, we want to use this tag.
                    telem.addData("has data", "true");
                    if (detection.id == tagID || detection.id == tagID+3) {
                        telem.addData("is correct", true);
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }

            if (targetFound) {
                break;
            }


        }

        telem.addData("DONE","D");
        telem.update();

        sleep(1000);
        if (targetFound) {
            bearningError = desiredTag.ftcPose.x;
            yawError = desiredTag.ftcPose.yaw;
            rangeError = desiredTag.ftcPose.range;
        }

        telem.addData("tag", tagID);
        if (targetFound) {
            telem.addData("range", desiredTag.ftcPose.range);
            telem.addData("bearing", desiredTag.ftcPose.bearing);
            telem.addData("yaw", desiredTag.ftcPose.yaw);
            telem.update();
            return;
        } else {
            telem.addData("\n>","No detections\n");
            telem.update();
        }

    }

    public void runOpMode() {

    }
    private void initAprilTag(HardwareMap hwM) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwM.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        telem.addData("vision", visionPortal);
        telem.update();
        if (visionPortal == null) {
            telem.addData("no vision", "womo womp");
            telem.update();
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telem.addData("WAITING", visionPortal.getCameraState());
                telem.update();
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
