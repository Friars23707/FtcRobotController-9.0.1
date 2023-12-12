package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.*;

public class mainCamera extends LinearOpMode {
    VisionPortal webcamOne;
    VisionPortal webcamTwo;
    TfodProcessor tfod = TfodProcessor.easyCreateWithDefaults();

    public mainCamera() {
        webcamOne = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        webcamTwo = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }

    //Detecting if there is a team prop
    public void propDetection() {
        boolean propIsDetected = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions(); // List of recognitions
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2; // What do they do??
            double y = (recognition.getTop() + recognition.getBottom()) / 2; // ^^^
            double propConfidence = recognition.getConfidence(); // Prop detection confidence

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
//do not put any code here
    }
}
