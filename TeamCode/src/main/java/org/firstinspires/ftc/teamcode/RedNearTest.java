package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedNearTest", group = "LogansTest")
public class RedNearTest extends LinearOpMode {
    AutonClass ac = new AutonClass();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ac.spikePlace();
        ac.encoderDrive(2, 2, 2, 2); //Move to board
        ac.boardPlace();
        ac.encoderDrive(2,2,2,2); //Move out of the way
    }

}