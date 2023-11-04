package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RedLeft")

public class RedLeft extends LinearOpMode {

    double locationOfProp = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    private PIDController controller;

    // public static double p=0.0005, i = 0, d=0;
    public double p = 0.0094, i = 0.04, d=0.0032;
    public static double f = 0.3;

    public static int target = 0;

    //private final double ticks_in_degrees = 0;
    private final double ticks_in_degrees = 22.4;
    private DcMotorEx lift_right;
    private DcMotorEx lift_left;


    private Servo left_intake;

    private Servo right_intake;

    private Servo right_claw;

    private Servo left_claw;
    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift_right = hardwareMap.get(DcMotorEx.class, "lift_right");
        lift_left = hardwareMap.get(DcMotorEx.class, "lift_left");

//        left_intake = hardwareMap.get(Servo.class, "left_intake");
//        right_intake = hardwareMap.get(Servo.class, "right_intake");

        lift_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lift_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        right_claw = hardwareMap.get(Servo.class, "right claw");
//        left_claw = hardwareMap.get(Servo.class, "left claw");
//        left_claw.setDirection(Servo.Direction.REVERSE);
//        right_intake.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35,-61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        // Postion 1 - traj1StrafeLeft, Traj1
        Trajectory traj1StrafeLeft = drive.trajectoryBuilder(startPose)
                .strafeLeft(-13)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj1StrafeLeft.end())
                .forward(-24)
                .build();

        Trajectory back = drive.trajectoryBuilder(traj1.end())
                .back(-6)
                .build();

        //Postion 2 - Traj2
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(-34.5)
                .build();

        Trajectory back1 = drive.trajectoryBuilder(traj2.end())
                .back(-3)
                .build();


        //Position 3 - traj3StrafeLeft ,traj3 ,traj4
        Trajectory traj3StrafeLeft = drive.trajectoryBuilder(startPose)
                .strafeLeft(-12)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj3StrafeLeft.end())
                .forward(-28)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(-15)
                .build();

        Trajectory back2 = drive.trajectoryBuilder(traj4.end())
                .back(-3)
                .build();


        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

//        right_claw.setPosition(1);
//        left_claw.setPosition(1);

        sleep(5000);
        getLocationOfProp();

        waitForStart();


//        while (opModeIsActive()) {
//            controller.setPID(p, i, d);
//            int armPos = lift_left.getCurrentPosition();
//            double pid = controller.calculate(armPos, target);
//            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
//
//            double power = pid + ff;
//
//            lift_left.setPower(power);
//            lift_right.setPower(power);
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.addData("Location of Prop", locationOfProp);
//            telemetry.update();
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }

//        left_claw.setPosition(1);
//        right_claw.setPosition(1);

        sleep(1000);

        // target = 100;

        sleep(4000);

        if (locationOfProp == 1) {
            drive.followTrajectory(traj1StrafeLeft);
            drive.followTrajectory(traj1);
            drive.followTrajectory(back);
        }
        if (locationOfProp == 2) {
            drive.followTrajectory(traj2);
            drive.followTrajectory(back1);

        }
        if (locationOfProp == 3) {
            drive.followTrajectory(traj3StrafeLeft);
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(traj4);
            drive.followTrajectory(back2);
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    private void getLocationOfProp() {
        if (cX<500) {
            locationOfProp = 1;
        }
        if (cX>=500 && cX<=800) {
            locationOfProp = 2;
        }
        if (cX>800) {
            locationOfProp=3;
        }
    }


}
