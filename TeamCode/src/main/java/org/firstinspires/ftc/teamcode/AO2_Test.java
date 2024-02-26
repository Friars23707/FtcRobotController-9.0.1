package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class AO2_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("waiting for", " start");
        telemetry.update();
        waitForStart();
        telemetry.addData("go ", "yippie");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        AutonOrientation2 ao2 = new AutonOrientation2(drive, telemetry);
        DistanceSensor distBR = hardwareMap.get(DistanceSensor.class, "dist_b_r");
        DistanceSensor distBL = hardwareMap.get(DistanceSensor.class, "dist_b_l");
        ao2.orientRobot(distBR, distBL, 2);
    }
}
