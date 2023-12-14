package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "RedNearTest", group = "LogansTest")
public class RedNearTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonClass ac = new AutonClass(hardwareMap);
        waitForStart();
        ac.spikePlace();
        ac.encoderDrive(2, 2, 2, 2); //Move to board
        ac.boardPlace();
        ac.encoderDrive(2,2,2,2); //Move out of the way
    }

}
