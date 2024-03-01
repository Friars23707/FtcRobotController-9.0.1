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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutonOrientation2 extends LinearOpMode {

    // Declare OpMode members.
    final private static ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive;
    public Telemetry telem;
    public AutonOrientation2(SampleMecanumDrive drive2, Telemetry telem2) {
        drive = drive2;
        telem = telem2;
    }

    public TrajectorySequence orientRobot(DistanceSensor distR, DistanceSensor distL, int spike) {
        // Initialize the hardware variables
        double distance = getDistance(distR, distL, spike);
        telem.addData("Distance", distance);
        telem.addData("Left", unit(distL));
        telem.addData("Right", unit(distR));
        telem.update();

        TrajectorySequence fix = drive.trajectorySequenceBuilder(new Pose2d())
                .back(distance - 12)
                .build();

        // Execute the strafing trajectory
        return fix;

    }

    public double getDistance(DistanceSensor r, DistanceSensor l, int spike) {
        if (spike%2 == 0) {
            return (unit(r) + unit(l)) / 2;
        } else if (spike%3 == 0) {
            return unit(r);
        } else {
            return unit(l);
        }
    }

    public void gripperCenter(DistanceSensor distC) {
        // Initialize the hardware variables
        double distance = unit(distC);
        telem.addData("Distance", distance);
        telem.update();

        Trajectory fix = drive.trajectoryBuilder(new Pose2d())
                .back(distance-0.5)
                .build();

        // Execute the strafing trajectory
        drive.followTrajectory(fix);
    }

    public double unit(DistanceSensor d) {
        return d.getDistance(DistanceUnit.INCH);
    }
    public void runOpMode() throws InterruptedException { } // needs to exist
}
