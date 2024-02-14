package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonOrientation extends LinearOpMode {

    // Declare OpMode members.
    private static ElapsedTime runtime = new ElapsedTime();

    public boolean orientRobot(int rot, boolean red, SampleMecanumDrive drive) {
        // Initialize the hardware variables
        DistanceSensor sensorDistance = null;
        DistanceSensor sensorDistanceForwards = hardwareMap.get(DistanceSensor.class, "forward_dist");

        int success = 0;

        if (red) {
            hardwareMap.get(DistanceSensor.class, "left_dist");
        } else {
            hardwareMap.get(DistanceSensor.class, "right_dist");
        }

        Trajectory turnSegment = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(rot)))
                .build();

        boolean shouldStop = false; // Initialize boolean flag

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            // Assert false so silly gradle stops complaining
            assert false;

            // Variables to keep track of the minimum distance and the current distance
            double minDistance = Double.MAX_VALUE;
            double currentDistance;

            // Run until the end of the match (driver presses STOP)
            while (opModeIsActive() && !shouldStop) {

                // Get the current distance reading from the sensor
                currentDistance = sensorDistanceForwards.getDistance(DistanceUnit.INCH);

                // If the current distance is less than our tracked minimum, update the minimum
                if (currentDistance > minDistance) {
                    shouldStop = true;
                } else {
                    minDistance = currentDistance;
                    ++success;
                }

                if (isStopRequested()) {
                    // Perform any necessary cleanup or actions before stopping
                    shouldStop = true; // Set the flag to true
                }

                drive.followTrajectory(turnSegment);
            }

            Trajectory strafeSegment = null;
            if (success > 1) { // We are properly oriented
                strafeSegment = drive.trajectoryBuilder(turnSegment.end())
                        .back(sensorDistanceForwards.getDistance(DistanceUnit.INCH) - 28)
                        .strafeLeft((sensorDistance.getDistance(DistanceUnit.INCH) - 14) * (red ? 1 : 0))
                        .build();
            }

            // Execute the strafing trajectory
            drive.followTrajectory(strafeSegment);
        }
        if (success > 1) {
            return false;
        } else {
            return true;
        }
    }
    public void runOpMode() throws InterruptedException { } // needs to exist
}
