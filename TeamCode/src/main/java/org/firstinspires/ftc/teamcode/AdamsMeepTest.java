package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AdamsMeepTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10, -60, Math.toRadians(-90)));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(-90)))
                .setReversed(false)
                .strafeTo(new Vector2d(25, -32))
                .turn(Math.toRadians(-90))
                .forward(20)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(50, -28))
                .build();

        drive.followTrajectorySequence(traj1);
    }
}
