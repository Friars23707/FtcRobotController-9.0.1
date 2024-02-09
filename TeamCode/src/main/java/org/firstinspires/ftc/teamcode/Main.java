package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@TeleOp(name="Main", group=".Primary")


public class Main extends LinearOpMode {

    int armPos = 0;


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

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        //        // ########################################################################################
        //        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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


            //Reverse Drive
            if (gamepad1.dpad_left) {
                reverseConst = 1;
            } else if (gamepad1.dpad_right) {
                reverseConst = -1;
            }

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

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            //The reverse constant is normally 1, retaining your original values
            //If you press dpad left it negates it which swaps all motor valus.
            leftFrontDrive.setPower(leftFrontPower*reverseConst);
            rightFrontDrive.setPower(rightFrontPower*reverseConst);
            leftBackDrive.setPower(leftBackPower*reverseConst);
            rightBackDrive.setPower(rightBackPower*reverseConst);

            /*Logans attempt at arm code
            Notes as of 11/3/23: arm works up and down, more power needed to reset
            11/4/23: updated & optimised code
            11/27/23: code left behind in favor of encoders

            int up = gamepad2.left_bumper ? 1 : 0;
            int down = gamepad2.right_bumper ? 1 : 0;
            float armPower = up - down;
            armPower /= 2;

            leftArm.setPower(armPower);
            rightArm.setPower(armPower);

            // Wrist Code
            float wristPower = gamepad2.left_trigger - gamepad2.right_trigger;
            wristMotor.setPower(wristPower);
            */

            /*
            Encoder based arm + wrist code
            0 = intake
            250 = home
            2,300 = score
            */

            /*GAMEPAD 2 CONTROLS

            Mode 0: Encoder / Main Mode | In this mode you move the arm and control the gripper
            Mode 1: Manual | In this mode the operator can manualy move the arm and re-zero it to correct any errors

             */

            if (gamepad2.dpad_down) { //INTAKE
                setArmPow(0.5);
                armTarget = 0;
                wristTarget = 0;
            } else if (gamepad2.dpad_left) { //SCORE-LOW
                setArmPow(0.4);
                armTarget = 2450;
                wristTarget = 1700;
            } else if (gamepad2.dpad_up) { //SCORE-MID-LOW
                setArmPow(0.4);
                armTarget = 2800;
                wristTarget = 1300;
            } else if (gamepad2.dpad_right) { //SCORE-LOW-LOW IS TOO LOW!
                setArmPow(0.4);
                armTarget = 3000;
                wristTarget = 1100;
            }

            /*
            if (gamepad2.dpad_down) { //INTAKE
                armTarget = 0;
                wristTarget = 0;
            } else if (gamepad2.dpad_left) { //HOME
                armTarget = 250;
                wristTarget = 0;
            } else if (gamepad2.dpad_up) { //SCORE
                armTarget = 2450;
                wristTarget = 1700;
            } else if (gamepad2.dpad_right) { //SCORE-LOW
                armTarget = 2800;
                wristTarget = 1300;
            }*/

            if (gamepad2.x) { //CLOSE
                leftGripperPos = 0.39;
            } else if (gamepad2.a) {
                leftGripperPos = 0.45;
            }
            gripperLeft.setPosition(leftGripperPos);

            if (gamepad2.y) { //CLOSE
                rightGripperPos = 0.4;
            } else if (gamepad2.b) {
                rightGripperPos = 0.3;
            }
            gripperRight.setPosition(rightGripperPos);


            if (gamepad2.left_stick_y != 0) {
                armTarget = leftArm.getCurrentPosition() + Math.round(gamepad2.left_stick_y*4);
            }
            if (gamepad2.right_stick_y != 0) {
                wristTarget = wristMotor.getCurrentPosition() + Math.round(gamepad2.right_stick_y*4);
            }


            if (gamepad2.right_bumper) {
                leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }



            //Set arms to the wanted position based on inputs
            leftArm.setTargetPosition(armTarget);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setTargetPosition(armTarget);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristMotor.setTargetPosition(wristTarget);
            wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            //DRONE SHOOTER // JOINT GAMEPAD CONTROLED
            if (gamepad1.left_bumper && gamepad2.left_bumper) {
                //ADD DRONE SERVO HERE
                shooter.setPosition(0.5);
            } else {
                shooter.setPosition(0.44);
            }
            telemetry.addData("DRONE: ", shooter.getPosition());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
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
