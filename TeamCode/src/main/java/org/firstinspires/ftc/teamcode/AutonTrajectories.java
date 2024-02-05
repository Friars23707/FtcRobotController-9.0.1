package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonTrajectories {
    public static TrajectorySequence setTrajectory(boolean red, boolean far, int rmark, int bmark) {
        TrajectorySequence trajectory = null;
        SampleMecanumDrive drive = null;

        //Red Near Left
        if (red && !far && rmark == 1) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
            //Red Near Center
        } else if (red && !far && rmark == 2) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                    .strafeLeft(20)
                    .strafeTo(new Vector2d(40, -23))
                    .turn(Math.toRadians(-90))
                    .forward(20)
                    .back(20)
                    .strafeTo(new Vector2d(42, -34))
                    .strafeRight(25)
                    .back(16)
                    .build();
            //Red Near Right
        } else if (red && !far && rmark == 3) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                    .setReversed(false)
                    .strafeTo(new Vector2d(25, -32))
                    .turn(Math.toRadians(-90))
                    .forward(20)
                    .waitSeconds(0.2)
                    .strafeTo(new Vector2d(50, -28))
                    .build();

            //Red Far Left
        } else if (red && far && rmark == 1) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-47, -50))
                    .turn(Math.toRadians(-90))
                    .strafeRight(30)
                    .strafeLeft(10)
                    .back(8)
                    .waitSeconds(0.2)
                    .back(7)
                    .strafeRight(25)
                    .back(75)
                    .strafeTo(new Vector2d(42, -29))
                    .strafeRight(20)
                    .back(16)
                    .build();
            //Red Far Center
        } else if (red && far && rmark == 2) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                    .back(65)
                    .forward(22)
                    .waitSeconds(0.2)
                    .back(12)
                    .turn(Math.toRadians(-90))
                    .back(75)
                    .strafeTo(new Vector2d(42, -34))
                    .strafeRight(25)
                    .back(16)
                    .build();
            //Red Far Right
        } else if (red && far && rmark == 3) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                    .strafeTo(new Vector2d(-50, -30))
                    .turn(Math.toRadians(90))
                    .forward(18)
                    .waitSeconds(0.2)
                    .back(15)
                    .strafeLeft(20)
                    .turn(Math.toRadians(180))
                    .back(65)
                    .strafeTo(new Vector2d(42, -40))
                    .strafeRight(31)
                    .back(16)
                    .build();

            //Blue Near Left
        } else if (!red && !far && bmark == 1) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
            //Blue Near Center
        } else if (!red && !far && bmark == 2) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
            //Blue Near Right
        } else if (!red && !far && bmark == 3) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();

            //Blue Far Left
        } else if (!red && far && bmark == 1) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
            //Blue Far Center
        } else if (!red && far && bmark == 2) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
            //Blue Far Right
        } else if (!red && far && bmark == 3) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        }
        return trajectory;
    }
}
