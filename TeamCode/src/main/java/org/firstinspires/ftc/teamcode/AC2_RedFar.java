package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

@Autonomous(name = "RedFar2", group = "AC_Final2")
public class AC2_RedFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException, OpModeManagerImpl.ForceStopException {
        AutonClass2 ac = new AutonClass2(hardwareMap, telemetry, true, true);
        while (!isStopRequested() && !isStarted()) {
            ac.setCUSTOM_DELAY(gamepad1);
            telemetry.addData("WAITING ", ac.CUSTOM_DELAY);
            telemetry.update();
        }
        waitForStart();

        ac.runProgram(0);
    }

}
