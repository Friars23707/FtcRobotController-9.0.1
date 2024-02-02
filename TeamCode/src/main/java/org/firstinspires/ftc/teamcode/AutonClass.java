package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;


public class AutonClass extends LinearOpMode {

    private HardwareMap hardwareMap;
    private Gamepad gamepad1;


    private ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive = null;

    public int redMark = 0;
    public int blueMark = 0;
    public boolean redAlliance;
    public boolean isFar;
    public double CUSTOM_DELAY = 0;
    public boolean CAN_DELAY = true;

    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor wristMotor = null;
    public Servo gripperLeft = null;

    public Servo gripperRight = null;
    public HardwareMap hwM;
    public Telemetry telem;


    TrajectorySequence trajMove1; //Move to drop pixel on spike mark
    TrajectorySequence trajMove2; //Move to center lane of tiles
    TrajectorySequence trajMove3; //Move in front of backboard.
    TrajectorySequence trajMove4; //Move to correct board spot
    TrajectorySequence trajMove5; //Move out of the way
    public AutonClass(HardwareMap no, Telemetry tm, boolean isRed, boolean s_isFar) {

        hwM = no;
        telem = tm;
        redAlliance = isRed;
        isFar = s_isFar;
        drive = new SampleMecanumDrive(no);
        drive.setPoseEstimate(new Pose2d());

        //Arm Motors
        leftArm = hwM.get(DcMotor.class, "arm_motor_left"); // E0
        rightArm = hwM.get(DcMotor.class, "arm_motor_right"); // E1
        wristMotor = hwM.get(DcMotor.class, "wrist_motor"); // E3

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        wristMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Gripper Servos
        gripperLeft = hwM.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = hwM.get(Servo.class, "right_gripper"); //Servo-E1

        gripperLeft.setPosition(0.39);
        gripperRight.setPosition(0.4);

    }

    public void setCUSTOM_DELAY() {
        if (gamepad1 == null) {
            return;
        }
        if (!gamepad1.y && !gamepad1.a) {
            CAN_DELAY = true;
        }
        if (!CAN_DELAY) {
            return;
        }
        if (gamepad1.y) {
            CUSTOM_DELAY += 1.0;
            CAN_DELAY = false;
        }
        if (gamepad1.a) {
            CUSTOM_DELAY -= 1.0;
            CAN_DELAY = false;
        }
    }

    public void runProgram(int spikeOveride) throws InterruptedException {
        runtime.reset();

        double timeToWait = CUSTOM_DELAY - runtime.seconds();

        if (spikeOveride != 0) {
            redMark = spikeOveride;
            blueMark = spikeOveride;
        } else {
            getSpikeMark();
        }

        telem.addData("spikes", redMark+":"+blueMark);
        telem.addData("WAITING ", timeToWait);
        telem.update();

        if (timeToWait > 0) {
            sleep((long) (timeToWait*1000));
        }

        /*
         * Create Traj 1+2, which places purple on floor and moves out of the way.
         */
        //LEFT SPIKE PLACE
        if ((redAlliance == false && blueMark == 1) || (redAlliance == true && redMark == 1)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(25)
                    .waitSeconds(0.1)
                    .turn(Math.toRadians(-90))
                    .forward(4)
                    .build();
            if (redAlliance) {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(4)
                        .waitSeconds(0.1)
                        .strafeLeft(22)
                        .turn(Math.toRadians(180))
                        .build();
            } else {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(5)
                        .waitSeconds(0.1)
                        .strafeRight(22)
                        .turn(Math.toRadians(180))
                        .build();
            }
        //CENTER SPIKE PLACE
        } else if ((redAlliance == false && blueMark == 2) || (redAlliance == true && redMark == 2)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(42)
                    .waitSeconds(2)
                    .build();
            if (redAlliance) {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(6)
                        .turn(Math.toRadians(-90))
                        .build();
            } else {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(6)
                        .turn(Math.toRadians(90))
                        .build();
            }
        //RIGHT SPIKE PLACE
        } else if ((redAlliance == false && blueMark == 3) || (redAlliance == true && redMark == 3)) {
            trajMove1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(25)
                    .waitSeconds(0.3)
                    .turn(Math.toRadians(90))
                    .forward(4)
                    .build();
            if (redAlliance) {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(3)
                        .strafeRight(23)
                        .build();
            } else {
                trajMove2 = drive.trajectorySequenceBuilder(trajMove1.end())
                        .back(3)
                        .strafeLeft(2)
                        .build();
            }
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
        //NEAR
        if (!isFar) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .back(26)
                    .build();
        //FAR
        } else if (isFar) {
            trajMove3 = drive.trajectorySequenceBuilder(trajMove2.end())
                    .back(71)
                    .build();
        }

        /*
         * Create Traj 4+5, first 4 moves into the correct backboard spot, then 5 moves away
         */
        if (redAlliance) {
            if (redMark == 1) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(30)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(24)
                        .waitSeconds(0.1)
                        .back(21)
                        .build();
            } else if (redMark == 2) {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(40)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(40)
                        .waitSeconds(0.1)
                        .back(24)
                        .build();
            } else {
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeLeft(24)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeRight(27)
                        .waitSeconds(0.1)
                        .back(16)
                        .build();
            }
        } else {
            if (blueMark == 1) { // Left backboard
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(33)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(36)
                        .waitSeconds(0.1)
                        .back(16)
                        .build();
            } else if (blueMark == 2) { // Center backboard
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(24)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(24)
                        .waitSeconds(0.1)
                        .back(16)
                        .build();
            } else { // Right backboard
                trajMove4 = drive.trajectorySequenceBuilder(trajMove3.end())
                        .strafeRight(17)
                        .build();
                trajMove5 = drive.trajectorySequenceBuilder(trajMove4.end())
                        .strafeLeft(19)
                        .waitSeconds(0.1)
                        .back(16)
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
        double timeToWait = CUSTOM_DELAY - runtime.seconds();

        EOCV_Pipe1 pipe = new EOCV_Pipe1(hwM, telemetry);
        while ((redMark == 0 || redMark == 4 || blueMark == 0 || blueMark == 4) && !isStopRequested()) {
            redMark = pipe.getFinal_Red();
            blueMark = pipe.getFinal_Blue();
            telem.addData("spikes", redMark+":"+blueMark);
            telem.addData("WAITING ", timeToWait);
            telem.update();
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