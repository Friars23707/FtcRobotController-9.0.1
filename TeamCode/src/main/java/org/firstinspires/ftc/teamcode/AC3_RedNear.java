package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

@Autonomous(name = "RedNear3", group = "AC_Final3")
public class AC3_RedNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException, OpModeManagerImpl.ForceStopException {
        AdamsAuton ac = new AdamsAuton(hardwareMap, telemetry, true, false);
        while (!isStopRequested() && !isStarted()) {
            ac.setCUSTOM_DELAY(gamepad1);
            telemetry.addData("WAITING ", ac.CUSTOM_DELAY);
            telemetry.update();
        }
        waitForStart();

        ac.runProgram(0);
    }

}
