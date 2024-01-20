package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(group = "DR")
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
    private Encoder leftEncoder = null;
    private Encoder frontEncoder = null;

    double axial = 0;  // Note: pushing stick forward gives negative value
    double lateral = 0;
    double yaw = 0;

    int armTarget = 0;
    int wristTarget = 0;
    double leftGripperTar = 0.39;
    double rightGripperTar = 0.4;

    int spikeMark = 0;
    int step = 0;

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
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

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

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back_drive"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        gripperLeft.setPosition(0.39);
        gripperRight.setPosition(0.4);
        telemetry.addData("Status", "Awaiting Spike Mark");
        telemetry.update();
        getSpikeMark();
        runtime.reset();
        int startingLeftEncoder = leftEncoder.getCurrentPosition();
        int startingFrontEncoder = frontEncoder.getCurrentPosition();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int leftTicks = leftEncoder.getCurrentPosition() - startingLeftEncoder;
            int frontTicks = frontEncoder.getCurrentPosition() - startingFrontEncoder;
            telemetry.addData("Status", "Going to mark "+spikeMark);
            telemetry.addData("Time:", runtime.seconds());
            telemetry.addData("Left Ticks", leftTicks);
            telemetry.addData("Front Ticks", frontTicks);
            telemetry.addData("Step", step);
            telemetry.addData("Vales", axial+":"+lateral+":"+yaw);
            telemetry.update();
            //333 encoder ticks is 1 inches
            //3000 encoder ticks is 90* turn

            if (step == 4) {
                axial = 0;
                lateral = 0;
                yaw = 0;
                spikeDrop();
                step = 5;
            } else if (spikeMark == 1) {

                if (step == 0) {
                    move(-0.3, 0,0 );
                } else if (step == 1) {
                    move(0,0,0.2);
                } else if (step == 2) {
                    move(0.3,0,0);
                } else {
                    move(0,0,0);
                }

            } else if (spikeMark == 2) {

                if (step == 0) {
                    move(-0.3, 0,0 );
                } else {
                    move(0,0,0);
                }

            } else if (spikeMark == 3) {

                if (step == 0) {
                    move(0, -0.3, 0);
                } else if (step == 1) {
                    move(-0.3, 0, 0);
                } else {
                    move(0, 0, 0);
                }

            }

            class MyBackgroudMethod extends Thread {

                @Override
                public void run() {
                    while (opModeIsActive()) {

                        if (spikeMark == 1) {

                            if (step == 0) {
                                if (leftTicks > 7300) {
                                    step = 1;
                                }
                            } else if (step == 1) {
                                if (frontTicks > 3100) {
                                    step = 2;
                                }
                            } else if (step == 2) {
                                if (leftTicks < 5000) {
                                    step = 4;
                                }
                            }

                        } else if (spikeMark == 2) {

                            if (leftTicks > inchesToTicks(39)) {
                                step = 4;
                            }

                        } else if (spikeMark == 3) {

                            if (step == 0) {
                                if (frontTicks > inchesToTicks(13)) {
                                    step = 1;
                                }
                            } else if (step == 1) {
                                if (leftTicks > inchesToTicks(33)) {
                                    step = 4;
                                }
                            }

                        }

                    }
                }

            }
            MyBackgroudMethod thread = new MyBackgroudMethod();
            thread.setDaemon(true);
            thread.start();

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
            if (axial != 0 || lateral != 0 || yaw != 0) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            } else {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
        }

    }

    public double inchesToTicks(double inches) {
        return inches*333;
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

    public void move(double ax, double lat, double ya) {
        axial = ax;
        lateral = lat;
        yaw = ya;
    }

    public void spikeDrop() throws InterruptedException {
        gripperLeft.setPosition(0.45);
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

    double wheelCircum = 1.69291339*Math.PI;
    public void inchesForward(double inches) throws InterruptedException {
        double currentTicks = leftEncoder.getCurrentPosition();
        double inchesToTicks = wheelCircum*inches*2000;
        move(-0.1,0,0);
        while (leftEncoder.getCurrentPosition()-currentTicks < inchesToTicks) {

            Thread.sleep(1);

        }
        move(0,0,0);
        step++;
    }

}