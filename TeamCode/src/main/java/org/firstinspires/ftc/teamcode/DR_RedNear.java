package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "DR")
public class DR_RedNear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor wristMotor = null;
    private Servo gripperLeft = null;
    private Servo gripperRight = null;
    private Servo shooter = null;

    double axial = 0;  // Note: pushing stick forward gives negative value
    double lateral = 0;
    double yaw = 0;

    int armTarget = 0;
    int wristTarget = 0;
    double leftGripperTar = 0.39;
    double rightGripperTar = 0.4;

    int spikeMark = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftArm = hardwareMap.get(DcMotor.class, "arm_motor_left"); // E0
        rightArm = hardwareMap.get(DcMotor.class, "arm_motor_right"); // E1
        gripperLeft = hardwareMap.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = hardwareMap.get(Servo.class, "right_gripper"); //Servo-E1
        wristMotor = hardwareMap.get(DcMotor.class, "wrist_motor"); // E3

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Awaiting Spike Mark");
        telemetry.update();
        getSpikeMark();
        telemetry.addData("Status", "Going to mark "+spikeMark);
        telemetry.update();
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.milliseconds() < 4) {
                move(1, 0, 0);
            } else if (runtime.seconds() < 8) {
                if (spikeMark == 1)  {
                    move(0,0, 1);
                } else if (spikeMark == 2)  {
                    move(0,0, 1);
                } else if (spikeMark == 3)  {
                    move(0,0, 1);
                }
            } else {
                move(0,0,0);
            }

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

    }

    public void getSpikeMark() throws InterruptedException {
        while (hardwareMap == null && !isStopRequested()) {
            wait(1);
        }
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hardwareMap, telemetry);
        while ((spikeMark == 0 || spikeMark == 4) && !isStopRequested()) {
            spikeMark = pipe.getFinal_Red(); //GET THE SPIKE MARK
            sleep(10);
        }
        pipe.stop();
    }

    public void move(int ax, int lat, int ya) {
        axial = (ax == 1) ? -0.2 : 0;
        lateral = (lat == 1) ? -0.2 : 0;
        yaw = (ya == 1) ? -0.2 : 0;
    }

}