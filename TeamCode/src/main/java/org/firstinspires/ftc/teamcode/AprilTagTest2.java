package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AprilTagTest2 extends LinearOpMode {
    April2 april2 = new April2();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry.addData("STARTED", "TRUE");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        april2.AprilTagNavigation(drive, hardwareMap, telemetry);
        april2.navigateToTag(2);
    }
}
