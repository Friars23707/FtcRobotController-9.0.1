package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

@Autonomous(name = "BlueFar3", group = "AC_Final2")
public class AC3_BlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException, OpModeManagerImpl.ForceStopException {
        AdamsAuton ac = new AdamsAuton(hardwareMap, telemetry, false, true);
        while (!isStopRequested() && !isStarted()) {
            ac.setCUSTOM_DELAY(gamepad1);
            telemetry.addData("WAITING ", ac.CUSTOM_DELAY);
            telemetry.update();
        }
        waitForStart();

        ac.runProgram(0);
    }

}
