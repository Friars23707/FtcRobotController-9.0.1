import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name="OdomAuto", group="Pushbot")
public class OdomAuto extends LinearOpMode {

    // Declare our motors
    // Make sure your ID's match your configuration
    Encoder leftEncoderWheel, rightEncoderWheel, backEncoderWheel;

    double cirucmfrence =  48 * Math.PI;

    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables
        leftEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "left_back_drive"));
        rightEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        backEncoderWheel = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get the current counts for each encoder
            int leftCount = leftEncoderWheel.getCurrentPosition();
            int rightCount = rightEncoderWheel.getCurrentPosition();
            int backCount = backEncoderWheel.getCurrentPosition();

            // Calculate the robot's position based on encoder counts
            // This will depend on your specific robot's configuration
            // Here is a simple example:
            double xPos = (leftCount + rightCount) / 2.0;
            double yPos = backCount;

            // Display the real time stats
            telemetry.addData("Encoder Counts", "Left: %d  Right: %d  Back: %d", leftCount, rightCount, backCount);
            telemetry.addData("Position", "X: %.2f  Y: %.2f", xPos, yPos);
            telemetry.update();
        }
    }
}
