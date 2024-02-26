package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonOrientation2 extends LinearOpMode {

    // Declare OpMode members.
    final private static ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive;
    public Telemetry telem;
    public AutonOrientation2(SampleMecanumDrive drive2, Telemetry telem2) {
        drive = drive2;
        telem = telem2;
    }

    public void orientRobot(DistanceSensor distR, DistanceSensor distL, int spike) {
        // Initialize the hardware variables
        double distance = getDistance(distR, distL, spike);
        telem.addData("Distance", distance);
        telem.addData("Left", distL.getDistance(DistanceUnit.INCH));
        telem.addData("Right", distR.getDistance(DistanceUnit.INCH));
        telem.update();

        Trajectory fix2 = drive.trajectoryBuilder(new Pose2d())
                .back(8)
                .build();

        while (distance > 13) {

            // Execute the strafing trajectory
            drive.followTrajectory(fix2);
            distance = getDistance(distR, distL, spike);

        }

        Trajectory fix = drive.trajectoryBuilder(new Pose2d())
                .back(distance - 12)
                .build();

        // Execute the strafing trajectory
        drive.followTrajectory(fix);

    }

    public double getDistance(DistanceSensor r, DistanceSensor l, int spike) {
        if (spike%2 == 0) {
            return (r.getDistance(DistanceUnit.INCH) + l.getDistance(DistanceUnit.INCH)) / 2;
        } else if (spike%3 == 0) {
            return r.getDistance(DistanceUnit.INCH);
        } else {
            return l.getDistance(DistanceUnit.INCH);
        }
    }

    public void gripperCenter(DistanceSensor distC) {
        // Initialize the hardware variables
        double distance = distC.getDistance(DistanceUnit.INCH);
        telem.addData("Distance", distance);
        telem.update();

        Trajectory fix = drive.trajectoryBuilder(new Pose2d())
                .back(distance-0.5)
                .build();

        // Execute the strafing trajectory
        drive.followTrajectory(fix);
    }
    public void runOpMode() throws InterruptedException { } // needs to exist
}
