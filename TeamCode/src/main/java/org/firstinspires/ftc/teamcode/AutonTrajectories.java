package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonTrajectories {

    public HardwareMap hwM;
    private DcMotor leftArm = hwM.get(DcMotor.class, "arm_motor_left"); // E0
    private DcMotor rightArm = hwM.get(DcMotor.class, "arm_motor_right"); // E1
    private DcMotor wristMotor = hwM.get(DcMotor.class, "wrist_motor"); // E3

    public Servo gripperLeft = hwM.get(Servo.class, "left_gripper"); //Servo-E0

    public Servo gripperRight = hwM.get(Servo.class, "right_gripper"); //Servo-E1

    public static TrajectorySequence setTrajectory(boolean red, boolean far, int rmark, int bmark, SampleMecanumDrive drive) {
        TrajectorySequence trajectory = null;

        //Red Near Left
        if (red && !far && rmark == 1) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(13)
                    .build();
            //Red Near Center
        } else if (red && !far && rmark == 2) {
        //  drive.setPoseEstimate(new Pose2d(10, -60, Math.toRadians(-90)));
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(0)))
                    .back(20)
                    .strafeTo(new Vector2d(40, -23))
                    .turn(Math.toRadians(-90))
                    .forward(20)
                    .addDisplacementMarker(() -> {
                        // Perform the "Drop" action here

                    })
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
            drive.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(-90)));
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


    public void dropSpike() throws InterruptedException {
        gripperLeft.setPosition(0.45);
        Thread.sleep(2000);
    }

    public void boardDrop() throws InterruptedException {
        leftArm.setPower(0.5);
        rightArm.setPower(0.5);
        wristMotor.setPower(0.5);

        leftArm.setTargetPosition(3100);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPosition(3100);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(1000);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Thread.sleep(4000);
        gripperRight.setPosition(0.3);
        Thread.sleep(1000);
        leftArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPosition(0);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
