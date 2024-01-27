package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="OdomAuto", group="Auto")
public class OdomAuto extends LinearOpMode {

    // Declare our motors
    // Make sure your ID's match your configuration
    Encoder leftEncoderWheel, rightEncoderWheel, backEncoderWheel;

    // double cirucmfrence = 48 * Math.PI;

    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables
        leftEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back_drive"));
        rightEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        backEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        double div = 39.7902778;
        double rotation = 0;

        double yPos = 0;
        double xPos = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int leftDefault = leftEncoderWheel.getCurrentPosition();
        int rightDefault = rightEncoderWheel.getCurrentPosition();
        int backDefault = backEncoderWheel.getCurrentPosition();

        int leftCountOld = 0;
        int rightCountOld = 0;
        int backCountOld = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int leftTicks = leftEncoderWheel.getCurrentPosition() - leftDefault;
            int rightTicks = rightEncoderWheel.getCurrentPosition() - rightDefault;
            int backTicks = backEncoderWheel.getCurrentPosition() - backDefault;

            // Get the current counts for each encoder
            int leftCount = leftTicks - leftCountOld;
            int rightCount = rightTicks - rightCountOld;
            int backCount = backTicks - backCountOld;

            // Calculate the robot's position based on encoder counts
            // This will depend on your specific robot's configuration
            // Here is a simple example:
            if (Math.abs(backCount) < 2) { // we are moving
                double localY = (leftCount + rightCount) / 2.0;
                double localX = backCount;
                yPos += localY * Math.cos(rotation) + localX * Math.sin(rotation);
                xPos += localX * Math.cos(rotation) + localY * Math.sin(rotation);
            } else { // we are rotating
                int rot = backCount;
                rotation += rot / div;
            }

            // Display the real time stats
            telemetry.addData("Rotation", "%d", rotation);
            telemetry.addData("Encoder Counts", "Left: %d  Right: %d  Back: %d", leftCount, rightCount, backCount);
            telemetry.addData("Position", "X: %.2f  Y: %.2f", xPos, yPos);
            telemetry.update();

            // Get the current counts for each encoder
            leftCountOld = leftTicks;
            rightCountOld = rightTicks;
            backCountOld = backTicks;
        }
    }
}
