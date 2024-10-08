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

        int result = 0;
        while (result == 0 || result == 4) {
            if (isStopRequested()) {
                break;
            }
            result = pipe.getFinal_Red();
            sleep(100);
        }
        pipe.stop();

        String poses= "";
        switch (result){
            case 1: {
                poses = "LEFT";
                break;
            }
            case 2: {
                poses = "CENTER";
                break;
            }
            case 3: {
                poses = "RIGHT";
                break;
            }
        }
        
        telemetry.addData("SPIKE", result+poses);
        telemetry.update();

        sleep(5000000);
    }
}
