package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EOCV_T1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hardwareMap, telemetry);
        waitForStart();
        sleep(500);
        double result = pipe.getFinal_Result();
        pipe.stop();
        sleep(5000000);
    }
}
