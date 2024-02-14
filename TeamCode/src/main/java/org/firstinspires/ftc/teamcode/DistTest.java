package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous

public class DistTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Stat", "Waiting for start");
        telemetry.update();
        waitForStart();
        DistanceSensor distc = hardwareMap.get(DistanceSensor.class, "dist_c");
        DistanceSensor distbl = hardwareMap.get(DistanceSensor.class, "dist_b_l");
        DistanceSensor distbr = hardwareMap.get(DistanceSensor.class, "dist_b_r");
        while (opModeIsActive()) {
            telemetry.addData("dist gripper (inches): ", distc.getDistance(DistanceUnit.INCH));
            telemetry.addData("dist b l (inches): ", distbl.getDistance(DistanceUnit.INCH));
            telemetry.addData("dist b r (inches): ", distbr.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
