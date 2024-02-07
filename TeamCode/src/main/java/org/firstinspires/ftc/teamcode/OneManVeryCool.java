package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="OneMan", group=".Primary")


public class OneManVeryCool extends LinearOpMode {

    int armPos = 0;
    int gp2Mode = 0;

    // Declare OpMode members for each of the 4 motors.
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

    @Override
    public void runOpMode() {

        double leftGripperPos = 0.3;
        double rightGripperPos = 0.3;
        int armTarget = 0;
        int wristTarget = 0;
        int reverseConst = 1;


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); //0
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive"); //1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //2
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //3

        //Arm Motors
        leftArm = hardwareMap.get(DcMotor.class, "arm_motor_left"); // E0
        rightArm = hardwareMap.get(DcMotor.class, "arm_motor_right"); // E1
        wristMotor = hardwareMap.get(DcMotor.class, "wrist_motor"); // E3

        //Gripper Servos
        gripperLeft = hardwareMap.get(Servo.class, "left_gripper"); //Servo-E0
        gripperRight = hardwareMap.get(Servo.class, "right_gripper"); //Servo-E1
        shooter = hardwareMap.get(Servo.class, "shooter"); //Servo-E2


        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        wristMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //GAMEPAD 1 CONTROLS

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x/3; //Slow down turns

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower*reverseConst);
            rightFrontDrive.setPower(rightFrontPower*reverseConst);
            leftBackDrive.setPower(leftBackPower*reverseConst);
            rightBackDrive.setPower(rightBackPower*reverseConst);


            //MODE SWAP
            if (gamepad1.back) {
                gp2Mode = 0;
            } else if (gamepad1.options) {
                gp2Mode = 1;
            }

            //MODE 0: ENCODER / MAIN MODE
            if (gp2Mode == 0) {

                if (gamepad1.dpad_down) { //INTAKE
                    setArmPow(0.5);
                    armTarget = 0;
                    wristTarget = 0;
                } else if (gamepad1.dpad_left) { //SCORE-LOW
                    setArmPow(0.4);
                    armTarget = 2450;
                    wristTarget = 1700;
                } else if (gamepad1.dpad_up) { //SCORE-MID-LOW
                    setArmPow(0.4);
                    armTarget = 2800;
                    wristTarget = 1300;
                } else if (gamepad1.dpad_right) { //SCORE-LOW-LOW IS TOO LOW!
                    setArmPow(0.4);
                    armTarget = 3000;
                    wristTarget = 1100;
                }


                if (gamepad1.x) { //CLOSE
                    leftGripperPos = 0.39;
                } else if (gamepad1.a) {
                    leftGripperPos = 0.45;
                }
                gripperLeft.setPosition(leftGripperPos);

                if (gamepad1.y) { //CLOSE
                    rightGripperPos = 0.4;
                } else if (gamepad1.b) {
                    rightGripperPos = 0.3;
                }
                gripperRight.setPosition(rightGripperPos);

            //MODE 1: MANUAL / ZERO-ING MODE
            } else {

                armTarget = leftArm.getCurrentPosition() + (gamepad1.x ? 50:0) + (gamepad1.a ? -50:0);
                wristTarget = wristMotor.getCurrentPosition() + (gamepad1.y ? 60:0) + (gamepad1.b ? -60:0);

                if (gamepad1.dpad_left) {
                    leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

            }

            //Set arms to the wanted position based on inputs
            leftArm.setTargetPosition(armTarget);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setTargetPosition(armTarget);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristMotor.setTargetPosition(wristTarget);
            wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            //DRONE SHOOTER // JOINT GAMEPAD CONTROLED
            if (gamepad1.left_bumper) {
                //ADD DRONE SERVO HERE
                shooter.setPosition(0.5);
            } else {
                shooter.setPosition(0.44);
            }
            telemetry.addData("DRONE: ", shooter.getPosition());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            String gp2String = "MANUAL"; if (gp2Mode == 0) {gp2String = "ENCODER";}
            telemetry.addData("GP2 Mode", gp2String+" | R-Const: "+reverseConst);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Pos = "+armTarget+" | ", "Left = "+leftArm.getCurrentPosition()+" | Right = "+rightArm.getCurrentPosition());
            telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
            telemetry.addData("Servos | Left:", gripperLeft.getPosition()+" Right: "+gripperRight.getPosition());
            telemetry.update();

            // Share the CPU.
            sleep(20);
        }
    }

    public void setArmPow(double pow) {
        //Set the power for all encoders ((LEFT AND RIGHT ARM MUST MATCH OR KABOOM))
        leftArm.setPower(pow);
        rightArm.setPower(pow);
        wristMotor.setPower(pow);
    }

}
