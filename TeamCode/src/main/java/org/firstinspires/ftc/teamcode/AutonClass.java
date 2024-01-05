package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;


public class AutonClass extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive = null;

    public int redMark = 0;
    public int blueMark = 0;
    public boolean redAlliance;
    public boolean isFar;


    public Servo gripperLeft = null;

    public Servo gripperRight = null;
    public HardwareMap hwM;


    TrajectorySequence trajMove1; //Move to drop pixel on spike mark
    TrajectorySequence trajMove2; //Move to center lane of tiles
    TrajectorySequence trajMove3; //Move in front of backboard.
    TrajectorySequence trajMove4; //Move to correct board spot
    TrajectorySequence trajMove5; //Move out of the way
    public AutonClass(HardwareMap no, boolean isRed, boolean s_isFar) {

        hwM = no;
        redAlliance = isRed;
        isFar = s_isFar;
        drive = new SampleMecanumDrive(no);

        //Gripper Servos
        gripperLeft = no.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = no.get(Servo.class, "right_gripper"); //Servo-E1

        gripperLeft.setPosition(0.2);
        gripperRight.setPosition(0.2);

    }

    public void runProgram(int spikeOveride) throws InterruptedException {

        if (spikeOveride != 0) {
            redMark = spikeOveride;
            blueMark = spikeOveride;
        } else {
            getSpikeMark();
        }

        /*
         * Create Traj 1+2, which places purple on floor and moves out of the way.
         */
        //LEFT SPIKE PLACE
        if ((redAlliance == false && blueMark == 1) || (redAlliance == true && redMark == 1)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(30)
                    .waitSeconds(0.3)
                    .turn(Math.toRadians(90))
                    .back(10)
                    .build();
            trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                    .forward(10)
                    .turn(Math.toRadians(-90))
                    .back(20)
                    .build();
        //CENTER SPIKE PLACE
        } else if ((redAlliance == false && blueMark == 2) || (redAlliance == true && redMark == 2)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(46)
                    .build();
            trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                    .back(4)
                    .build();
        //RIGHT SPIKE PLACE
        } else if ((redAlliance == false && blueMark == 3) || (redAlliance == true && redMark == 3)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(30)
                    .waitSeconds(0.3)
                    .turn(Math.toRadians(-90))
                    .back(10)
                    .build();
            trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                    .forward(10)
                    .turn(Math.toRadians(90))
                    .back(20)
                    .build();
        //FAILSAFE
        } else {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3).build();
            trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                    .back(3).build();
        }

        /*
         * Create Traj 3, which moves to the middle tiles in front of the board.
         */
        //RED NEAR
        if (!isFar && redAlliance) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .turn(Math.toRadians(-90))
                    .back(26)
                    .build();
        //RED FAR
        } else if (isFar && redAlliance) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .turn(Math.toRadians(-90))
                    .back(26+48)
                    .build();
        //BLUE NEAR
        } else if (!isFar && !redAlliance) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .turn(Math.toRadians(90))
                    .back(26)
                    .build();
        //BLUE FAR
        } else if (isFar && !redAlliance) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .turn(Math.toRadians(90))
                    .back(26+48)
                    .build();
        }

        /*
         * Create Traj 4+5, first 4 moves into the correct backboard spot, then 5 moves away
         */
        if (redAlliance) {
            if (redMark == 1) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(20)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(20)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            } else if (redMark == 2) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(24)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(24)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            } else {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(28)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(28)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            }
        } else {
            if (redMark == 1) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(20)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(20)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            } else if (redMark == 2) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(24)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(24)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            } else {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(28)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(28)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            }
        }


        /*
         * Run all of the actual trajectories.
         */
        drive.followTrajectorySequence(trajMove1);
        spikeDrop();
        drive.followTrajectorySequence(trajMove2);
        drive.followTrajectorySequence(trajMove3);
        drive.followTrajectorySequence(trajMove4);
        boardDrop();
        drive.followTrajectorySequence(trajMove5);



    }

    public void getSpikeMark() throws InterruptedException {
        while (hwM == null && !isStopRequested()) {
            wait(1);
        }
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hwM, telemetry);
        while ((redMark == 0 || redMark == 4 || blueMark == 0 || blueMark == 4) && !isStopRequested()) {
            redMark = pipe.getFinal_Red();
            blueMark = pipe.getFinal_Blue();
            sleep(10);
        }
        pipe.stop();
    }

    public void spikeDrop() {
        gripperLeft.setPosition(0.6);
    }

    public void boardDrop() {
        gripperRight.setPosition(0.5);
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