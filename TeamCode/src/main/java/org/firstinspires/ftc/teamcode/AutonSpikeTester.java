package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonSpikeTester", group = "LogansTest")
public class AutonSpikeTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        int ISRED = 0;
        int SPIKE = 0;
        int ISFAR = 0;

        while (opModeIsActive()) {
            ISRED = 0;
            SPIKE = 0;
            ISFAR = 0;
            telemetry.addData("Which spike mark do you want to go to?","");
            telemetry.addData("Left", "X");
            telemetry.addData("Center", "Y");
            telemetry.addData("Right", "B");
            telemetry.update();
            while (SPIKE == 0) {
                if (gamepad1.x) {
                    SPIKE = 1;
                } else if (gamepad1.y) {
                    SPIKE = 2;
                } else if (gamepad1.b) {
                    SPIKE = 3;
                }
            }

            telemetry.addData("Which side are you on?","");
            telemetry.addData("Red", "Left D-Pad");
            telemetry.addData("Blue", "Right D-Pad");
            telemetry.update();
            while (ISRED == 0) {
                if (gamepad1.dpad_left) {
                    ISRED = 1;
                } else if (gamepad1.dpad_right) {
                    ISRED = 2;
                }
            }

            telemetry.addData("Where are you?","");
            telemetry.addData("Near", "Up D-Pad");
            telemetry.addData("Far", "Down D-Pad");
            telemetry.update();
            while (ISFAR == 0) {
                if (gamepad1.dpad_up) {
                    ISFAR = 1;
                } else if (gamepad1.dpad_down) {
                    ISFAR = 2;
                }
            }

            boolean isBoolRed = ISRED == 1 ? true : false;
            boolean isBoolFar = ISFAR == 1 ? false : true;
            AutonClass ac = new AutonClass(hardwareMap, telemetry, isBoolRed, isBoolFar);
            ac.runProgram(SPIKE);

        }



    }

}