package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class AprilTagTest extends LinearOpMode {
    AprilTagPipeline aprilTagP;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry.addData("STARTED", "TRUE");
        telemetry.update();
        aprilTagP = new AprilTagPipeline(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        aprilTagP.initVision(hardwareMap);
        aprilTagP.orient(drive, 2, hardwareMap);
    }
}
