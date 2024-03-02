package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
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

    public void orient(SampleMecanumDrive drive, int tagID, HardwareMap hwM) {
        telem.addData("PLSPLS", yawError);
        telem.addData("PLSPLS2", bearningError);
        telem.update();
        getErrors(tagID, hwM);
        if (bearningError == 0.0 || yawError == 0.0 || rangeError == 0.0) { return; }
        sleep(1000);
        TrajectorySequence fix3 = drive.trajectorySequenceBuilder(new Pose2d())
                //.strafeRight(yawError) //yawError
                //.back(rangeError-12) //rangeError-12
                .turn(Math.toRadians(bearningError * 5)) //Math.toRadians(bearningError)
                .build();
        drive.followTrajectorySequence(fix3);
    }

    public void getErrors(int tagID, HardwareMap hwM) {

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        // Initialize the Apriltag Detection process
        initAprilTag(hwM);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

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


        }

        if (targetFound) {
            bearningError = desiredTag.ftcPose.bearing;
            yawError = desiredTag.ftcPose.yaw;
            rangeError = desiredTag.ftcPose.range;
        }

        telem.addData("tag", tagID);
        if (targetFound) {
            telem.addData("range", desiredTag.ftcPose.range);
            telem.addData("bearing", desiredTag.ftcPose.bearing);
            telem.addData("yaw", desiredTag.ftcPose.yaw);
        } else {
            telem.addData("\n>","No detections\n");
        }
        telem.update();

    }

    public void runOpMode() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        // Initialize the Apriltag Detection process
        initAprilTag(hardwareMap);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    // Yes, we want to use this tag.
                    if (detection.id == 5 || detection.id == 2) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    }
                }
            }


        }
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
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
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
