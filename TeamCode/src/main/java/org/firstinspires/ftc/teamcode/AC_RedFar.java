package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "RedFar", group = "AC_Final")
public class AC_RedFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonClass ac = new AutonClass(hardwareMap, telemetry, true, true);
        waitForStart();

        ac.runProgram(0);
    }

}
