package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

@TeleOp
public class TestBedArmYippie extends LinearOpMode {

    DcMotor motor;
    Servo gripperLeft;

    int targetPos = 0;

    double leftGripperPos = 0;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "coreHex");
        gripperLeft = hardwareMap.get(Servo.class, "sTest");

        // Reset the encoder during initialization
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Set the motor's target position to 300 ticks


        // Switch to RUN_TO_POSITION mode

        // Start the motor moving by setting the max velocity to 200 ticks per second
        motor.setPower(20);

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {

            if (gamepad1.x) {
                targetPos = -50;
            } else if (gamepad1.a) {
                targetPos = 0;
            }
            motor.setTargetPosition(targetPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //TEST SERVO
            if (gamepad2.x) {
                leftGripperPos = 0.3;
            } else if (gamepad2.a) {
                leftGripperPos = 0.4;
            }
            gripperLeft.setPosition(leftGripperPos);

            telemetry.addData("velocity", motor.getPower());
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addData("is at target", !motor.isBusy());
            telemetry.addData("tartgetpos", targetPos);
            telemetry.addData("SERVO POS:", gripperLeft.getPosition());
            telemetry.update();



        }
    }

}
