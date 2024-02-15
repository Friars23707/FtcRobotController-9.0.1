package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
LEFT GRIPPER
0.39 close
0.45 open

RIGHT GRIPPER
0.3 open
0.4 close
 */

@TeleOp
public class gripperTesting extends LinearOpMode {
    private Servo gripperLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        gripperLeft = hardwareMap.get(Servo.class, "right_gripper");
        double gpos = 0.1;
        while (!isStopRequested()) {
            Thread.sleep(3000);
            gpos += 0.1;
            if (gpos > 0.6) {
                gpos = 0.1;
            }
            telemetry.addData("gpos", gpos);
            telemetry.update();
            gripperLeft.setPosition(gpos);
        }
    }
}
