package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "BlueNear", group = "AC_Final")
public class AC_BlueNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonClass ac = new AutonClass(hardwareMap, telemetry, false, false);
        while (!isStopRequested() && !isStarted()) {
            ac.setCUSTOM_DELAY();
            telemetry.addData("WAITING ", ac.CUSTOM_DELAY);
            telemetry.update();
        }
        waitForStart();

        ac.runProgram(0);
    }

}
