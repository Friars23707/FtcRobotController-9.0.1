package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.camera2.*;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.hardware.camera2.*;
import android.media.Image;
import android.media.ImageReader;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.core.content.ContextCompat;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.nio.ByteBuffer;
import java.util.Arrays;

@TeleOp(name = "FriarCameraAPI3", group = "FTC")
public class FriarCameraAPI3 extends LinearOpMode {
    private ImageReader imageReader;
    private CameraDevice cameraDevice;
    private CameraCaptureSession captureSession;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the image reader
        imageReader = ImageReader.newInstance(1280, 720, ImageFormat.YUV_420_888, 2);

        // Get the camera manager
        CameraManager cameraManager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);

        // Open the camera
        String cameraId = null; // Use the first camera
        try {
            cameraId = cameraManager.getCameraIdList()[0];
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }
        if (ContextCompat.checkSelfPermission(hardwareMap.appContext, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        try {
            cameraManager.openCamera(cameraId, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(@NonNull CameraDevice camera) {
                    cameraDevice = camera;

                    // Create a capture session
                    try {
                        camera.createCaptureSession(Arrays.asList(imageReader.getSurface()), new CameraCaptureSession.StateCallback() {
                            @Override
                            public void onConfigured(@NonNull CameraCaptureSession session) {
                                captureSession = session;

                                // Start capturing images
                                CaptureRequest.Builder requestBuilder = null;
                                try {
                                    requestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                                } catch (CameraAccessException e) {
                                    throw new RuntimeException(e);
                                }
                                requestBuilder.addTarget(imageReader.getSurface());
                                try {
                                    captureSession.setRepeatingRequest(requestBuilder.build(), null, null);
                                } catch (CameraAccessException e) {
                                    throw new RuntimeException(e);
                                }
                            }

                            @Override
                            public void onConfigureFailed(@NonNull CameraCaptureSession session) {
                                // Handle configuration failure...
                            }
                        }, null);
                    } catch (CameraAccessException e) {
                        throw new RuntimeException(e);
                    }
                }

                @Override
                public void onDisconnected(@NonNull CameraDevice camera) {
                    // Handle camera disconnection...
                }

                @Override
                public void onError(@NonNull CameraDevice camera, int error) {
                    // Handle camera error...
                }
            }, null);
        } catch (CameraAccessException e) {
            throw new RuntimeException(e);
        }


        waitForStart();

        while (opModeIsActive()) {
            Image image = imageReader.acquireLatestImage();
            if (image != null) {
                try {
                    int pixelColor = getPixelColor(image, 100, 100);
                    telemetry.addData("Pixel Color", pixelColor);
                    telemetry.update();
                } finally {
                    image.close();
                }
            }
            sleep(10);
        }
    }

    private int getPixelColor(Image image, int x, int y) {
        Image.Plane[] planes = image.getPlanes();
        ByteBuffer buffer = planes[0].getBuffer();
        int pixelStride = planes[0].getPixelStride();
        int rowStride = planes[0].getRowStride();
        int rowOffset = planes[0].getRowStride() * y;
        int pixelOffset = rowOffset + pixelStride * x;
        int r = buffer.get(pixelOffset);
        int g = buffer.get(pixelOffset + pixelStride);
        int b = buffer.get(pixelOffset + pixelStride * 2);
        return android.graphics.Color.rgb(r, g, b);
    }
}
