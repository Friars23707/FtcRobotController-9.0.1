package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.media.Image;
import android.media.ImageReader;
import android.util.Size;
import android.view.Surface;

import android.os.Handler;
import android.os.HandlerThread;

import androidx.annotation.NonNull;
import androidx.core.content.ContextCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.ByteBuffer;
import java.util.Arrays;

@Autonomous(name="FriarCameraAPI2", group="Linear OpMode")
public class FriarCameraAPI2 extends LinearOpMode {

    private CameraDevice cameraDevice;
    private Bitmap bitmap;
    private int[] pixels;
    private ImageReader imageReader;
    private HandlerThread handlerThread; // Declare HandlerThread at class level

    public void runOpMode() {
        waitForStart();

        CameraManager cameraManager = (CameraManager) hardwareMap.appContext.getSystemService(Context.CAMERA_SERVICE);
        try {
            String cameraId = cameraManager.getCameraIdList()[0];
            CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(cameraId);
            Size previewSize = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(ImageFormat.YUV_420_888)[0];

            pixels = new int[previewSize.getWidth() * previewSize.getHeight()];
            bitmap = Bitmap.createBitmap(previewSize.getWidth(), previewSize.getHeight(), Bitmap.Config.ARGB_8888);

            imageReader = ImageReader.newInstance(previewSize.getWidth(), previewSize.getHeight(), ImageFormat.YUV_420_888, 2);
            handlerThread = new HandlerThread("Camera2HandlerThread"); // Initialize HandlerThread
            handlerThread.start();
            Handler handler = new Handler(handlerThread.getLooper());

            imageReader.setOnImageAvailableListener(new ImageReader.OnImageAvailableListener() {
                @Override
                public void onImageAvailable(ImageReader reader) {
                    Image image = reader.acquireLatestImage();
                    if (image != null) {
                        ByteBuffer buffer = image.getPlanes()[0].getBuffer();
                        byte[] bytes = new byte[buffer.remaining()];
                        buffer.get(bytes);
                        decodeYUV420SP(pixels, bytes, previewSize.getWidth(), previewSize.getHeight());
                        bitmap.setPixels(pixels, 0, previewSize.getWidth(), 0, 0, previewSize.getWidth(), previewSize.getHeight());
                        image.close();
                    }
                }
            }, handler);

            if (ContextCompat.checkSelfPermission(hardwareMap.appContext, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
                return;
            }
            cameraManager.openCamera(cameraId, new CameraDevice.StateCallback() {
                @Override
                public void onOpened(@NonNull CameraDevice camera) {
                    cameraDevice = camera;
                    startPreview();
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
        } catch (CameraAccessException e) {
            telemetry.addData("Error", "Camera access exception: " + e.getMessage());
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("color", getRedColor(640, 360)); // center of camera for now
            telemetry.update();
            sleep(1000);
        }

        stopCam();
    }

    private void startPreview() {
        try {
            CaptureRequest.Builder previewRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            previewRequestBuilder.addTarget(imageReader.getSurface());

            cameraDevice.createCaptureSession(Arrays.asList(imageReader.getSurface()), new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        session.setRepeatingRequest(previewRequestBuilder.build(), null, null);
                    } catch (CameraAccessException e) {
                        telemetry.addData("Error", "Failed to start camera preview.");
                        telemetry.update();
                    }
                }

                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession session) {
                }
            }, null);
        } catch (CameraAccessException e) {
            telemetry.addData("Error", "Failed to start camera preview.");
            telemetry.update();
        }
    }

    public void stopCam() {
        handlerThread.quitSafely();
        cameraDevice.close();
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

    private void decodeYUV420SP(int[] rgb, byte[] yuv420sp, int width, int height) {
        final int frameSize = width * height;

        for (int j = 0, yp = 0; j < height; j++) {
            int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
            for (int i = 0; i < width; i++, yp++) {
                int y = (0xff & ((int) yuv420sp[yp])) - 16;
                if (y < 0) y = 0;
                if ((i & 1) == 0) {
                    v = (0xff & yuv420sp[uvp++]) - 128;
                    u = (0xff & yuv420sp[uvp++]) - 128;
                }

                int y1192 = 1192 * y;
                int r = (y1192 + 1634 * v);
                int g = (y1192 - 833 * v - 400 * u);
                int b = (y1192 + 2066 * u);

                if (r < 0) r = 0; else if (r > 262143) r = 262143;
                if (g < 0) g = 0; else if (g > 262143) g = 262143;
                if (b < 0) b = 0; else if (b > 262143) b = 262143;

                rgb[yp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
            }
        }
    }
}
