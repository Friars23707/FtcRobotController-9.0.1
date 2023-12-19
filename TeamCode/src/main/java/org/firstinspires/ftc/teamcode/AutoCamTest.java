package org.firstinspires.ftc.teamcode;

import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.IOException;

@Autonomous(name="AutoCamTest", group="bruh")
public class AutoCamTest extends OpMode {
    private Camera camera;
    private SurfaceTexture surfaceTexture = new SurfaceTexture(10);

    public void init() {
        // List<Camera.Size> sizes = cam.getParameters().getSupportedPreviewSizes();
        camera = Camera.open();
        Camera.Parameters parameters = camera.getParameters();
        parameters.setPreviewSize(1280, 720);
        parameters.setPreviewFormat(ImageFormat.NV21);
        camera.setParameters(parameters);

        try {
            camera.setPreviewTexture(surfaceTexture);
        } catch (IOException e) {
            e.printStackTrace();
        }

        camera.startPreview();
    }

    @Override
    public void loop() {
        telemetry.addData("PIXEL", getPixelColor(300, 100));
        telemetry.update();
    }

    public int getPixelColor(int x, int y) {
        Camera.Size size = camera.getParameters().getPreviewSize();
        byte[] data = new byte[size.width * size.height];
        camera.addCallbackBuffer(data);
        int index = y * size.width + x;
        int pixel = data[index] & 0xFF;

        return pixel;
    }
}
