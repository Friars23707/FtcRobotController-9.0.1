package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.camera2.*;
import android.media.Image;
import android.media.ImageReader;
import android.util.Size;
import androidx.annotation.NonNull;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;

@Autonomous(name = "cameraProcessor", group = "Linear OpMode")
public class cameraProcessor extends LinearOpMode {

    private CameraDevice camera;
    private Bitmap bitmap;
    private int[] pixels;

    public void runOpMode() {
        waitForStart();
        sleep(10);

        CameraManager manager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);
        try {
            String[] cameraIdList = manager.getCameraIdList();
            String cameraId = "";
            if (cameraIdList.length > 0) {
                cameraId = cameraIdList[0];
                // Continue with your code
            } else {
                // Handle the case when no camera is available
                stopCam();
            }


            if (ContextCompat.checkSelfPermission(hardwareMap.appContext, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                manager.openCamera(cameraId, new CameraDevice.StateCallback() {
                    @Override
                    public void onOpened(@NonNull CameraDevice camera) {
                        cameraProcessor.this.camera = camera;
                        createCameraPreviewSession();
                    }

                    @Override
                    public void onDisconnected(@NonNull CameraDevice camera) {
                        camera.close();
                    }

                    @Override
                    public void onError(@NonNull CameraDevice camera, int error) {
                        camera.close();
                    }
                }, null);
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }

        while (opModeIsActive()) {
            // 720p = 1280 x 720
            telemetry.addData("color", getRedColor(640, 360)); // center of camera for now
            telemetry.update();
            sleep(1000);
        }
    }

    private void createCameraPreviewSession() {
        try {
            Size previewSize = new Size(1280, 720); // replace with your desired preview size
            ImageReader reader = ImageReader.newInstance(previewSize.getWidth(), previewSize.getHeight(), ImageFormat.YUV_420_888, 2);
            reader.setOnImageAvailableListener(new ImageReader.OnImageAvailableListener() {
                @Override
                public void onImageAvailable(ImageReader reader) {
                    Image image = reader.acquireLatestImage();
                    if (image != null) {
                        // process image
                        image.close();
                    }
                }
            }, null);

            CaptureRequest.Builder builder = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            builder.addTarget(reader.getSurface());

            camera.createCaptureSession(Collections.singletonList(reader.getSurface()), new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        session.setRepeatingRequest(builder.build(), null, null);
                    } catch (CameraAccessException e) {
                        e.printStackTrace();
                    }
                }

                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession session) {
                }
            }, null);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    public void stopCam() { // when no more need for camera close it so no more memory gets leaked
        camera.close();
    }

    public int getColor(int x, int y) {
        return bitmap.getPixel(x, y);
    }

    public int getRedColor(int x, int y) {
        int color = bitmap.getPixel(x, y);
        return Color.red(color);
    }

    public int getBlueColor(int x, int y) {
        int color = bitmap.getPixel(x, y);
        return Color.blue(color);
    }
}
