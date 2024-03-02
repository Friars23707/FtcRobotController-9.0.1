package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class April2 extends LinearOpMode {

    private SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    AprilTagProcessor aprilTag;

    public void AprilTagNavigation(SampleMecanumDrive drive, HardwareMap hardwareMap, Telemetry telemetry) {
        // Create the AprilTag processor and assign it to a variable.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        telemetry.addData("IDK", "PLS");
        telemetry.update();
    }

    public void navigateToTag(int targetTagId) throws InterruptedException {
        // Assuming you have an AprilTagDetector instance called "aprilTag"
        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        telemetry.addData("target", !targetFound);
        telemetry.update();

        while (!isStopRequested() && !targetFound) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("target", "HI");
            telemetry.addData("Size", currentDetections.size());
            telemetry.update();


            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("ID", detection.id);
                telemetry.update();
                if (isStopRequested()) {
                    break;
                }
                if (detection.id == targetTagId || detection.id == targetTagId + 3) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }
        }

        if (targetFound) {
            // Extract position information from AprilTagPoseFtc
            double x = desiredTag.ftcPose.x; // X position
            double y = desiredTag.ftcPose.y; // Y position

            // Calculate desired position in front of the tag (12 inches away)
            double distanceToTag = 12.0; // inches
            Pose2d desiredPosition = new Pose2d(x + distanceToTag, y, Math.atan2(y, x));

            // Create a trajectory to move to the desired position
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(desiredPosition)
                    .build();

            // Execute the trajectory
            drive.followTrajectory(trajectory);

            // Update telemetry with relevant information
            telemetry.addData("Target Tag ID", targetTagId);
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.update();
        } else {
            telemetry.addData("Error", "Tag not found!");
            telemetry.update();
        }
    }
    public void runOpMode() throws InterruptedException {} //lol
}
