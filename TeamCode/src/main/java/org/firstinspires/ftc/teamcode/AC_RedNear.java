package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "RedNear", group = "AC_Final")
public class AC_RedNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException, OpModeManagerImpl.ForceStopException {
        AutonClass ac = new AutonClass(hardwareMap, telemetry, true, false);
        while (!isStopRequested() && !isStarted()) {
            ac.setCUSTOM_DELAY();
            telemetry.addData("WAITING ", ac.CUSTOM_DELAY);
            telemetry.update();
        }
        waitForStart();

        ac.runProgram(0);
    }

}
