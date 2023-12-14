package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "AutoCamTest", group = "CameraTest")
public class AutoCamTest extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;

    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;
    VisionPortal webcamOne;
    private void initTfod() {
        tfod = TfodProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            webcamOne = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
        webcamOne = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, tfod);
        }
    }
    @Override
    public void runOpMode() {

    }
}
