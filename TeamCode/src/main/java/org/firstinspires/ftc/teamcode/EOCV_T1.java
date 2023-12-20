package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class EOCV_T1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        EOCV_Pipe1 pipe = new EOCV_Pipe1(hardwareMap, telemetry);
        waitForStart();
        pipe.stop();
        int result = pipe.getFinal_Result();
        telemetry.addData("RESULT", result);
        telemetry.update();
    }
}
