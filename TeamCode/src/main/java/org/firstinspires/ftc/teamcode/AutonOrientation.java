package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonOrientation extends LinearOpMode {

    // Declare OpMode members.
    private static ElapsedTime runtime = new ElapsedTime();

    public double orientRobot(int speed, boolean red) {
        // Initialize the hardware variables
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        DistanceSensor sensorDistance = null;
        if (red) {
            hardwareMap.get(DistanceSensor.class, "left_dist");
        } else {
            hardwareMap.get(DistanceSensor.class, "right_dist");
        }

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            // Get the current distance reading from the sensor
            double distance = sensorDistance.getDistance(DistanceUnit.CM);

            // Variables to keep track of the minimum distance and the current distance
            double minDistance = Double.MAX_VALUE;
            double currentDistance;

            // Run until the end of the match (driver presses STOP)
            while (opModeIsActive() && !isStopRequested()) {

                // Get the current distance reading from the sensor
                currentDistance = sensorDistance.getDistance(DistanceUnit.CM);

                // If the current distance is less than our tracked minimum, update the minimum
                if (currentDistance < minDistance) {
                    minDistance = currentDistance;
                }

                // If the current distance starts to increase, we've found the minimum distance point
                if (currentDistance > minDistance) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    return currentDistance;  // exit the loop
                } else {
                    // Otherwise, continue turning
                    frontLeftDrive.setPower(speed);
                    frontRightDrive.setPower(-speed);
                    backLeftDrive.setPower(speed);
                    backRightDrive.setPower(-speed);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        return sensorDistance.getDistance(DistanceUnit.CM);  // exit the loop
    }
    public void runOpMode() throws InterruptedException {
        /*This just needs to exist so
        that we can use linearOpMode vars*/
    }
}
