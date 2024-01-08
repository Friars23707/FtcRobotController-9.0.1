package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class AC_Near_Tesr extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive = null;

    public int redMark = 0;
    public int blueMark = 0;

    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor wristMotor = null;
    public Servo gripperLeft = null;

    public Servo gripperRight = null;

    TrajectorySequence trajMove1;
    TrajectorySequence trajMove2;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        //Arm Motors
        leftArm = hardwareMap.get(DcMotor.class, "arm_motor_left"); // E0
        rightArm = hardwareMap.get(DcMotor.class, "arm_motor_right"); // E1
        wristMotor = hardwareMap.get(DcMotor.class, "wrist_motor"); // E3

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        wristMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Gripper Servos
        gripperLeft = hardwareMap.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = hardwareMap.get(Servo.class, "right_gripper"); //Servo-E1

        gripperLeft.setPosition(0.39);
        gripperRight.setPosition(0.4);

        waitForStart();
        getSpikeMark();
        telemetry.addData("spikes", redMark+":"+blueMark);
        telemetry.update();

        trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(33)
                .waitSeconds(0.1)
                .back(5)
                .build();
        trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                .back(35)
                .waitSeconds(0.1)
                .build();

        drive.followTrajectorySequence(trajMove1);
        spikeDrop();
        drive.followTrajectorySequence(trajMove2);
        boardDrop();

    }

    public void getSpikeMark() throws InterruptedException {
        while (hardwareMap == null && !isStopRequested()) {
            wait(1);
        }
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hardwareMap, telemetry);
        while ((redMark == 0 || redMark == 4 || blueMark == 0 || blueMark == 4) && !isStopRequested()) {
            redMark = pipe.getFinal_Red();
            blueMark = pipe.getFinal_Blue();
            sleep(10);
        }
        pipe.stop();
    }

    public void spikeDrop() throws InterruptedException {
        gripperLeft.setPosition(0.45);
        Thread.sleep(2000);
    }

    public void boardDrop() throws InterruptedException {
        leftArm.setPower(0.5);
        rightArm.setPower(0.5);
        wristMotor.setPower(0.5);

        leftArm.setTargetPosition(2800);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setTargetPosition(2800);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(1300);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Thread.sleep(3000);
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