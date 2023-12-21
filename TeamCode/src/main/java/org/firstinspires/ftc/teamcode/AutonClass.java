package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class AutonClass extends LinearOpMode {
    /* Declare OpMode members. */

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightArmMotor = null;
    private DcMotor leftArmMotor = null;

    private ElapsedTime runtime = new ElapsedTime();

    public SampleMecanumDrive drive = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    final double DRIVE_SPEED = 0.6;
    final double TURN_SPEED = 0.5;

    //HardwareMap yes;
    public AutonClass(HardwareMap no) {

        drive = new SampleMecanumDrive(no);

    }

    public void spikePlace(Telemetry telemetry) {
        telemetry.addData("Spike", "Yes");
        telemetry.update();

        TrajectorySequence trajTurnRight = drive.trajectorySequenceBuilder(new Pose2d())
                .back(53)
                .waitSeconds(0.3)
                .turn(Math.toRadians(-90))
                .back(82)
                .waitSeconds(0.3)
                .strafeLeft(20)
                .build();
        drive.followTrajectorySequence(trajTurnRight);
    }

    public void boardPlace() {
        telemetry.addData("j", "h");
    }

    //Basic RR Functions for all classes to use.
    public void rrFoward(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrBack(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .back(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrStrafeLeft(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrStrafeRight(int inches) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(inches)
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }
    public void rrTurn(int degrees) {
        TrajectorySequence simpleTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(degrees))
                .build();
        drive.followTrajectorySequence(simpleTraj);
    }

    public void runOpMode() throws InterruptedException {
        /*This just needs to exist so
        that we can use linearOpMode vars*/

    }
}



        /* Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        rightArmMotor = hardwareMap.get(DcMotor.class, "arm_motor_right");
        leftArmMotor = hardwareMap.get(DcMotor.class, "arm_motor_left");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/