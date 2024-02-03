package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class AutonClass2 extends LinearOpMode {

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


    TrajectorySequence trajMoveU;
    public AutonClass2(HardwareMap no, Telemetry tm, boolean isRed, boolean s_isFar) {

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

        if (spikeOveride != 0) {
            redMark = spikeOveride;
            blueMark = spikeOveride;
        } else {
            getSpikeMark();
        }

        double timeToWait = CUSTOM_DELAY - runtime.seconds();
        telem.addData("spikes", redMark+":"+blueMark);
        telem.addData("WAITING ", timeToWait);
        telem.update();

        if (timeToWait > 0) {
            sleep((long) (timeToWait*1000));
        }


        //Red Near Left
        if (redAlliance && !isFar && redMark == 1) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Red Near Center
        } else if (redAlliance && !isFar && redMark == 2) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Red Near Right
        } else if (redAlliance && !isFar && redMark == 3) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();

        //Red Far Left
        } else if (redAlliance && isFar && redMark == 1) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Red Far Center
        } else if (redAlliance && isFar && redMark == 2) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Red Far Right
        } else if (redAlliance && isFar && redMark == 3) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();

        //Blue Near Left
        } else if (!redAlliance && !isFar && blueMark == 1) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Blue Near Center
        } else if (!redAlliance && !isFar && blueMark == 2) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Blue Near Right
        } else if (!redAlliance && !isFar && blueMark == 3) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();

        //Blue Far Left
        } else if (!redAlliance && isFar && blueMark == 1) {
        trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                .back(3)
                .build();
        //Blue Far Center
        } else if (!redAlliance && isFar && blueMark == 2) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
        //Blue Far Right
        } else if (!redAlliance && isFar && blueMark == 3) {
            trajMoveU = drive.trajectorySequenceBuilder(new Pose2d())
                    .back(3)
                    .build();
    }



        /*
         * Run the actual trajectories.
         */
        drive.followTrajectorySequence(trajMoveU);



    }

    public void getSpikeMark() throws InterruptedException {
        while (hwM == null && !isStopRequested()) {
            wait(1);
        }
        double timeToWait;

        EOCV_Pipe1 pipe = new EOCV_Pipe1(hwM, telemetry);
        while (((redAlliance && (redMark == 0 || redMark == 4)) || (!redAlliance && (blueMark == 0 || blueMark == 4))) && !isStopRequested()) {
            redMark = pipe.getFinal_Red();
            blueMark = pipe.getFinal_Blue();
            timeToWait = CUSTOM_DELAY - runtime.seconds();
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