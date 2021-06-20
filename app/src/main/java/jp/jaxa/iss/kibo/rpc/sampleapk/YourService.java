package jp.jaxa.iss.kibo.rpc.sampleapk;
import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static org.opencv.android.Utils.matToBitmap;

import org.json.JSONObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.lang.Math;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        api.startMission();
        api.flashlightControlFront(0.6f);
        api.moveTo(new Point(11.21, -9.8, 4.79), new Quaternion(0f, 0f, -0.707f, 0.707f), true);

        wait(25000);

        //Get Mat Full Image
        Mat image = new Mat();

        double p = 0;
        double x = 0;
        double y = 0;
        double z = 0;

        Map<Integer, Double> x_ARpos = new HashMap<>();
        x_ARpos.put(0, 0.0d);

        Map<Integer, Double> y_ARpos = new HashMap<>();
        y_ARpos.put(0, 0.0d);

        //Crop Only QR And Read QR
        for (int i = 0; i < 3; i++) {
            image = api.getMatNavCam();

            int s_x = 590;
            int s_w = 780 - 590;
            int s_y = 555;
            int s_h = 775 - 555;
            Mat crop = image.submat(s_y, s_y + s_h,
                    s_x, s_x + s_w);

            Bitmap qrImage = Bitmap.createBitmap(s_w, s_h, Bitmap.Config.ARGB_8888);
            matToBitmap(crop, qrImage, false);

            int[] pixels = new int[qrImage.getWidth() * qrImage.getHeight()];
            qrImage.getPixels(pixels, 0, qrImage.getWidth(), 0, 0, qrImage.getWidth(), qrImage.getHeight());
            LuminanceSource luminance = new RGBLuminanceSource(qrImage.getWidth(), qrImage.getHeight(), pixels);
            BinaryBitmap postQrImage = new BinaryBitmap(new HybridBinarizer(luminance));

            QRCodeReader reader = new QRCodeReader();
            try {
                com.google.zxing.Result result = reader.decode(postQrImage);
                String contents = result.getText();
                JSONObject object = new JSONObject(contents);
                p = object.getDouble("p");
                x = object.getDouble("x");
                y = object.getDouble("y");
                z = object.getDouble("z");
                Log.d("QR[STATUS]", "QR: " + contents);
                api.sendDiscoveredQR(contents);
                break;
            } catch (Exception e) {
            }
        }

        //Crop Only AR
        Log.d("AR[STATUS]", "STARTING CROPING");

        int c_x = 365;
        int c_w = 800 - c_x;
        int c_y = 720;
        int c_h = 960 - c_y;
        Mat crop_ar = image.submat(c_y, c_y + c_h,
                c_x, c_x + c_w);
        Log.d("AR[STATUS]", "CROPING DONE ");
        Log.d("AR[STATUS]", "STARTING PADDING ");

        //Padding
        Mat crop_pad_ar = new Mat();
        Core.copyMakeBorder(crop_ar, crop_pad_ar, 0, 100, 0, 0, Core.BORDER_CONSTANT);

        //Distortion Matrix
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        double[] cam = {567.229305,     0.0,        659.077221 - c_x - 29.077221,
                0.0,    574.192915,     517.007571 - c_y - 17.007571,
                0,          0,                      1
        };
        double[] coef = {-0.216247, 0.03875, -0.010157, 0.001969, 0.0};

        cameraMatrix.put(0, 0, cam);
        distCoeffs.put(0, 0, coef);

        Log.d("AR[STATUS]", "PADDING DONE");
        Log.d("AR[STATUS]", "STARTING UNDISTORTION");

        //Start Distorting AR
        Mat AR = new Mat();
        Imgproc.undistort(crop_pad_ar, AR, cameraMatrix, distCoeffs);

        // add some comment
        int newAR_x = 0;
        int newAR_y = 0;

        Log.d("AR[STATUS]", "UNDISTORTION DONE");

        int offset_x = c_x + newAR_x;
        int offset_y = c_y + newAR_y;

        //Read AR And Calculate Average Ar Position
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat markerIds = new Mat();
        List<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(AR, dictionary, corners, markerIds);
        Log.d("AR[STATUS]", "markerIDs: " + markerIds.dump());
        for (int i = 0; i < corners.size(); i++) { //Get All Corner And Calculate Average AR Position
            Mat corner;
            corner = corners.get(i);
            Log.d("AR[STATUS]", "VALUES: " + corner.dump());
            int marker = (int) markerIds.get(i, 0)[0];
            double tl_x = corner.get(0, 0)[0];
            double tl_y = corner.get(0, 0)[1];
            double tr_x = corner.get(0, 1)[0];
            double tr_y = corner.get(0, 1)[1];
            double bl_x = corner.get(0, 2)[0];
            double bl_y = corner.get(0, 2)[1];
            double br_x = corner.get(0, 3)[0];
            double br_y = corner.get(0, 3)[1];
            double x_pos = (tl_x + tr_x + bl_x + br_x) / 4;
            double y_pos = (tl_y + tr_y + bl_y + br_y) / 4;
            Log.d("[AR]STATUS", "X POS: " + x_pos);
            Log.d("[AR]STATUS", "Y POS: " + y_pos);
            x_ARpos.put(marker, x_pos);
            y_ARpos.put(marker, y_pos);
        }

        //Calculate Average Target Point In Pixel
        double x_tar_pos = (x_ARpos.get(1) + x_ARpos.get(2) + x_ARpos.get(3) + x_ARpos.get(4)) / 4.0;
        double y_tar_pos = (y_ARpos.get(1) + y_ARpos.get(2) + y_ARpos.get(3) + y_ARpos.get(4)) / 4.0;
        Log.d("AR[STATUS]", "X TARGET POSITION: " + x_tar_pos);
        Log.d("AR[STATUS]", "Y TARGET POSITION: " + y_tar_pos);
        Log.d("AR[STATUS]", "X TARGET POSITION WITH OFFSET: " + (x_tar_pos + offset_x));
        Log.d("AR[STATUS]", "Y TARGET POSITION WITH OFFSET: " + (y_tar_pos + offset_y));

        //Calculate Actual Target Position
        double scale = 0.0013; //<-- 0.1485/100 <-- 0.1485 CM
        //Changing scale for accuracy
        double pixelAr_x = (x_tar_pos + offset_x) - (1280d / 2d);//find pixel in picture to pixel in camera
        double pixelAr_y = (y_tar_pos + offset_y) - (960d / 2d);// ''------------------------------------''
        double xtar_point = (pixelAr_x * scale);//find position in picture
        double ytar_point = (pixelAr_y * scale);// ''-------------------''

        //Got Picture From Point A
        double nav_cam_xOffset = (11.21) + xtar_point + (-0.0826);//finally find actual target position
        double nav_cam_yOffset = (-9.8) - 0.6673 - 0.1177;
        double nav_cam_zOffset = (4.79) + ytar_point + (-0.0442);// ''------------------------------''
        Log.d("AR[STATUS]", "NAV CAM OFFSET X: " + nav_cam_xOffset + " NAV CAM OFFSET Y: " + nav_cam_yOffset + " NAV CAM OFFSET Z: " + nav_cam_zOffset);

        //Calculate Euler Angle
        double relative_px = nav_cam_xOffset - (x + 0.0572);
        double relative_py = nav_cam_yOffset - (y + 0.1302);
        double relative_pz = nav_cam_zOffset - (z + (-0.1111));
        //SAME EQUATION ISN'T INVERT YET//
        double result = Math.sqrt((relative_px * relative_px) + (relative_py * relative_py));
        double yaw = Math.atan2(relative_px, -relative_py);
        double pitch = Math.atan2(result, relative_pz);
        double roll = 0;
        Log.d("EULER[STATUS]", "YAW: " + (yaw * 180) / Math.PI);
        Log.d("EULER[STATUS]", "PITCH: " + (pitch * 180) / Math.PI);

        //Calculate Euler To Quaternion
        double rad_90 = Math.PI / 2;
        double Ztar_RAD = pitch - rad_90; //From Start To Target //Inverted Pitch X --> Z
        double Xtar_RAD = yaw; // "--------------" // Z --> X, Value Stay The Same
        Log.d("QUATERNION[STATUS]", "YAW: " + (Ztar_RAD * 180) / Math.PI);
        Log.d("QUATERNION[STATUS]", "PITCH: " + (Xtar_RAD * 180) / Math.PI);

        double c1 = Math.cos(Ztar_RAD / 2);
        double c2 = Math.cos(Xtar_RAD / 2);
        double c3 = Math.cos(roll / 2);

        double s1 = Math.sin(Ztar_RAD / 2);
        double s2 = Math.sin(Xtar_RAD / 2);
        double s3 = Math.sin(roll / 2);

        double c1c2 = c1 * c2;
        double s1s2 = s1 * s2;

        double qw = c1c2 * c3 - s1s2 * s3; //W
        double qx = c1c2 * s3 + s1s2 * c3; //X
        double qy = s1 * c2 * c3 + c1 * s2 * s3; //Y
        double qz = c1 * s2 * c3 - s1 * c2 * s3; //Z

        //Calculate Cross Vector Quaternion
        //Docking --> A --> A' --> Target
        double a_qw = 0.707; //a1
        double a_qx = 0; //b1
        double a_qy = 0; //c1
        double a_qz = -0.707; //d1
        double new_qw = (a_qw * qw) - (a_qx * qx) - (a_qy * qy) - (a_qz * qz);
        double new_qx = (a_qw * qx) + (a_qx * qw) + (a_qy * qz) - (a_qz * qy);
        double new_qy = (a_qw * qy) - (a_qx * qz) + (a_qy * qw) + (a_qz * qx);
        double new_qz = (a_qw * qz) + (a_qx * qy) - (a_qy * qx) + (a_qz * qw);

        //Moving to A' And Rotate To Target
        int movecaseA = 0;
        if (p == 1 || p == 2 || p == 3 || p == 4 || p == 8) {
            movecaseA = 1;
        } else if (p == 5 || p == 6) {
            movecaseA = 2;
        } else if (p == 7){
            movecaseA = 3;
        } else {
            Log.d("Moving[STATUS]", "STATUS: ERROR");
        }
        switch (movecaseA) { //11.21, -9.8, 4.79
            case 1:
                api.moveTo(new Point(x, y, z), new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                break;
            case 2:
                api.moveTo(new Point(11.21 - 0.8, -9.8, 4.79), new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                api.moveTo(new Point(10.41, -9.8, 4.79 + 0.6),new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                api.moveTo(new Point(x, y, z),  new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                break;
            case 3:
                api.moveTo(new Point(11.21 + 0.3, -9.8, 4.79), new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                api.moveTo(new Point(11.51, -9.8, 4.79 + 0.6), new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                api.moveTo(new Point(x, y, z), new Quaternion((float) new_qx, (float) new_qy, (float) new_qz, (float) new_qw), true);
                break;
        }

        //Firing Laser
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);

        //Move To Point B
        int movecaseB = 0;
        if (p == 1 || p == 8) {
            movecaseB = 1;
            Log.d("Moving[STATUS]", "B STATUS: USING CASE 1");
        } else if (p == 2 || p == 3 || p == 4) {
            movecaseB = 2;
            Log.d("Moving[STATUS]", "B STATUS: USING CASE 2");
        } else if (p == 5 || p == 6) {
            movecaseB = 3;

        } else if (p == 7){
            movecaseB = 4;
        } else {
            Log.d("Moving[STATUS]", "STATUS: ERROR");
        }
        switch (movecaseB) {
            case 1:
                api.moveTo(new Point(x, y, z - 0.3), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(x - 0.6, y, z - 0.3), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(10.6, -8.0, 4.5), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                break;
            case 2:
                api.moveTo(new Point(x - 0.6, y, z - 0.8), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(10.6, -8.0, 4.5), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                break;
            case 3:
                api.moveTo(new Point(x - 0.5, y, z), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(10.6, -8.0, 4.5), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                break;
            case 4:
                api.moveTo(new Point(11.51, -9.8, 5.39), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(11.51, -9.8, 5.39), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(11.51, -9.8, 4.79), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(11.21 - 0.7, -9.8, 4.79 - 0.4), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                api.moveTo(new Point(10.6, -8.0, 4.5), new Quaternion(0f, 0f, -0.707f, 0.707f), true);
                break;
        }

        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        runPlan1();
    }

    @Override
    protected void runPlan3() {
        runPlan1();
    }

    public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw) {
        Result result;
        int count = 0, max = 3;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion(qx, qy, qz, qw);
        do {
            result = api.moveTo(point, quaternion, true);
            count++;
        } while (!result.hasSucceeded() && count < max);
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}