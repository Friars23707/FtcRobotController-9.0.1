package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class AutonClass extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive = null;

    public int redMark = 0;
    public int blueMark = 0;
    public boolean redAlliance;


    public Servo gripperLeft = null;

    public Servo gripperRight = null;

    public AutonClass(HardwareMap no, boolean isRed) {

        redAlliance = isRed;
        drive = new SampleMecanumDrive(no);

        //Gripper Servos
        gripperLeft = no.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = no.get(Servo.class, "right_gripper"); //Servo-E1

        gripperLeft.setPosition(0.2);
        gripperRight.setPosition(0.2);

    }

    public void spikePlace() {

        getSpikeMark();

        TrajectorySequence trajMove;
        if ((redAlliance == false && blueMark == 1) || (redAlliance == true && redMark == 1)) {
            trajMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(27)
                    .waitSeconds(0.3)
                    .turn(Math.toRadians(-90))
                    .back(19)
                    .build();
        } else {
            trajMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3).build();
        }
        drive.followTrajectorySequence(trajMove);
        spikeDrop();

    }

    public void getSpikeMark() {
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hardwareMap, telemetry);
        while ((redMark == 0 || redMark == 4 || blueMark == 0 || blueMark == 4) && !isStopRequested()) {
            redMark = pipe.getFinal_Red();
            blueMark = pipe.getFinal_Blue();
            sleep(10);
        }
    }

    public void boardPlace() {
        telemetry.addData("j", "h");
    }

    public void spikeDrop() {
        gripperLeft.setPosition(0.6);
    }

    public void boardDrop() {
        gripperRight.setPosition(0.5);
    }

    //Basic RR Functions for all classes to use.
    public void rrFoward(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrBack(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .back(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrStrafeLeft(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrStrafeRight(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrTurn(int degrees) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(degrees))
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }

    public void runOpMode() throws InterruptedException {
        /*This just needs to exist so
        that we can use linearOpMode vars*/

    }
}



/* Initialize the drive system variables.
leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
rightArmMotor = hardwareMap.get(DcMotor.class, "arm_motor_right");
leftArmMotor = hardwareMap.get(DcMotor.class, "arm_motor_left");

// To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
// When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
// Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/