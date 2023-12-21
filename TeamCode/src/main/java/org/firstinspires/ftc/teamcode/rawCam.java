package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.hardware.Camera.Size;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="rawCam", group="Linear OpMode")
public class rawCam extends LinearOpMode {

    private Camera camera;
    private Bitmap bitmap;
    private int[] pixels;

    public void runOpMode() {
        waitForStart();
        camera = Camera.open();
        Size previewSize = camera.getParameters().getPreviewSize();

        pixels = new int[previewSize.width * previewSize.height];
        bitmap = Bitmap.createBitmap(previewSize.width, previewSize.height, Bitmap.Config.ARGB_8888);

        camera.setPreviewCallback(new PreviewCallback() {
            @Override
            public void onPreviewFrame(byte[] data, Camera camera) {
                decodeYUV420SP(pixels, data, previewSize.width, previewSize.height);
                bitmap.setPixels(pixels, 0, previewSize.width, 0, 0, previewSize.width, previewSize.height);
            }
        });

        Camera.Parameters parameters = camera.getParameters();
        parameters.setPreviewFormat(ImageFormat.NV21);
        camera.setParameters(parameters);

        camera.startPreview();

        while (opModeIsActive()) {
            // 720p = 1280 x 720
            telemetry.addData("color", getRedColor(5, 5)); // center of camera for now
            telemetry.addData("bit width", bitmap.getWidth());
            telemetry.addData("bit height", bitmap.getHeight());
            telemetry.update();
            sleep(1000);
        }

    }

    public void stopCam() { // when no more need for camera close it so no more memory gets leaked
        camera.stopPreview();
        camera.release();
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

