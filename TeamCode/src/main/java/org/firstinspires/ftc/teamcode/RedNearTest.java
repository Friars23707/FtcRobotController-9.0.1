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

        //Move to board
            ac.rrTurn(-90);
            ac.rrBack(15);
            ac.rrStrafeLeft(28);

        ac.boardPlace();

        //Move out of the way
            ac.rrStrafeLeft(29);
    }

}
