package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Config

@Autonomous(name = "RedRIght")

public class RedRIght extends LinearOpMode {

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

    public static int lowH = 100;

    public static int lowS = 100;

    public static int lowV = 100;

    public static int highH = 180;

    public static int highS = 255;

    public static int highV = 255;

    private Servo armangle;

    private Servo basket;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx lift_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        DcMotorEx lift_motor_right = hardwareMap.get(DcMotorEx.class, "lift_right");
        armangle = hardwareMap.get(Servo.class, "ArmAngleServo");


        Servo left_intake = hardwareMap.get(Servo.class, "left_intake");
        Servo right_intake = hardwareMap.get(Servo.class, "right_intake");

        basket = hardwareMap.get(Servo.class, "basket");


        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory left1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(5)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .forward(-30)
                                .build();

        Trajectory left3 = drive.trajectoryBuilder(new Pose2d(17, -30), Math.toRadians(180))
                        .forward(-11)
                                .build();

        Trajectory left31 = drive.trajectoryBuilder(left3.end())
                .forward(8)
                .build();



        Trajectory left4 = drive.trajectoryBuilder(left31.end().plus(new Pose2d(0,0, Math.toRadians(-180))))
                        .strafeRight(2)
                                .build();

        Trajectory left41 = drive.trajectoryBuilder(left4.end())
                .forward(-34)
                .build();

        Trajectory left415 = drive.trajectoryBuilder(left41.end())
                .forward(-3)
                    .build();


        Trajectory left42 = drive.trajectoryBuilder(left415.end())
                .forward(6)
                .build();

        Trajectory left5 = drive.trajectoryBuilder(left42.end())
                .strafeRight(-12)
                .build();

        Trajectory left6 = drive.trajectoryBuilder(left5.end())
                .forward(-9)
                .build();



        Trajectory mid1 = drive.trajectoryBuilder(startPose)
                        .forward(-31.5)
                                .build();
        Trajectory mid2 = drive.trajectoryBuilder(mid1.end())
                        .back(-10)
                                .build();

        Trajectory mid3 = drive.trajectoryBuilder(mid2.end().plus(new Pose2d(0,0,Math.toRadians(-93))))
                .forward(-32)
                .build();



        Trajectory mid315 = drive.trajectoryBuilder(mid3.end())
                .strafeRight(2)
                .build();

        Trajectory mid31 = drive.trajectoryBuilder(mid315.end())
                .forward(-11)
                .build();

//        Trajectory mid305 = drive.trajectoryBuilder(mid31.end())
//                .strafeRight(4)
//                .build();

        Trajectory mid32 = drive.trajectoryBuilder(mid31.end())
                .forward(4)
                .build();

        Trajectory mid4 = drive.trajectoryBuilder(mid32.end())
                .strafeRight(-20)
                .build();

//        Trajectory mid5 = drive.trajectoryBuilder(mid4.end())
//                .back(5)
//                .build();

//        Trajectory mid3 = drive.trajectoryBuilder(mid2.end())
//                .forward(33)
//                .build();
//
//        Trajectory mid4 = drive.trajectoryBuilder(mid3.end())
//                .strafeLeft(48)
//                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPose)
                        .strafeRight(-11)
                                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .forward(-24)
                                .build();

        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .back(-10)
                        .build();

        Trajectory right4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(22)
                .build();

        Trajectory right405 = drive.trajectoryBuilder(right4.end())
                .strafeRight(3)
                .build();

        Trajectory right407 = drive.trajectoryBuilder(right405.end())
                .back(7.5)
                .build();

        Trajectory right41 = drive.trajectoryBuilder(right407.end())
                .forward(4)
                .build();

        Trajectory right5 = drive.trajectoryBuilder(right41.end())
                .strafeRight(-20)
                        .build();

//        Trajectory right6 = drive.trajectoryBuilder(right5.end())
//                .back(6)
//                        .build();


//        Trajectory right3 = drive.trajectoryBuilder(right2.end())
//                        .back(-25)
//                                .build();
//
//        Trajectory right4 = drive.trajectoryBuilder(right3.end())
//                        .strafeLeft(36)
//                                .build();

//        lift_motor_left.setPower(0);
//        lift_motor_right.setPower(0);
//
        lift_motor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        lift_motor_right.setTargetPosition(0);
//        lift_motor_left.setTargetPosition(0);

//        right_claw.setPosition(1);







        waitForStart();

        getLocationOfProp();
        sleep(1000);


        if (locationOfProp == 1) {
            drive.followTrajectory(left1);
            drive.followTrajectory(left2);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(left3);




            drive.followTrajectory(left31);

            drive.turn(Math.toRadians(-180));


            drive.followTrajectory(left4);

            drive.followTrajectory(left41);

            lift_motor_left.setTargetPosition(105);
            lift_motor_right.setTargetPosition(105);

            lift_motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_motor_right.setPower(0.7);
            lift_motor_left.setPower(0.7);





//            right_claw.setPosition(0.1);



            drive.followTrajectory(left415);

            basket.setPosition(0.5);

            sleep(3000);

            drive.followTrajectory(left42);

            basket.setPosition(0);
            sleep(1000);
            left_intake.setPosition(0.7);
            right_intake.setPosition(0.7);



            lift_motor_left.setTargetPosition(0);
            lift_motor_right.setTargetPosition(0);

//            lift_motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            lift_motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_motor_right.setPower(0.5);
            lift_motor_left.setPower(0.5);




            drive.followTrajectory(left5);




//            drive.followTrajectory(left6);
        }

        if(locationOfProp == 2) {
            drive.followTrajectory(mid1);
            drive.followTrajectory(mid2);
            drive.turn(Math.toRadians(-93));
            drive.followTrajectory(mid3);

            drive.followTrajectory(mid315);


            lift_motor_left.setTargetPosition(125);
            lift_motor_right.setTargetPosition(125);

            lift_motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_motor_right.setPower(0.7);
            lift_motor_left.setPower(0.7);

            sleep(2000);


            drive.followTrajectory(mid31);



            basket.setPosition(0.5);

            sleep(4000);

            drive.followTrajectory(mid32);
            basket.setPosition(0);

            left_intake.setPosition(0.7);
            right_intake.setPosition(0.7);

            lift_motor_left.setTargetPosition(0);
            lift_motor_right.setTargetPosition(0);

            lift_motor_right.setPower(0.7);
            lift_motor_left.setPower(0.7);

            drive.followTrajectory(mid4);
//            drive.followTrajectory(mid5);
        }

        if(locationOfProp == 3) {
            drive.followTrajectory(right1);
            drive.followTrajectory(right2);
            drive.followTrajectory(right3);
            drive.turn(Math.toRadians(-90));

            lift_motor_left.setTargetPosition(125);
            lift_motor_right.setTargetPosition(125);

            lift_motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift_motor_right.setPower(0.7);
            lift_motor_left.setPower(0.7);

            sleep(3000);

            drive.followTrajectory(right4);

            drive.followTrajectory(right405);

            drive.followTrajectory(right407);

            armangle.setPosition(0.65);

            sleep(1000);
            basket.setPosition(0.5);
            sleep(2000);

            drive.followTrajectory(right41);

            armangle.setPosition(0.5);


            basket.setPosition(0);

            left_intake.setPosition(0.7);
            right_intake.setPosition(0.7);

            lift_motor_left.setTargetPosition(0);
            lift_motor_right.setTargetPosition(0);



            lift_motor_right.setPower(0.7);
            lift_motor_left.setPower(0.7);


            drive.followTrajectory(right5);
//            drive.followTrajectory(right6);

        }

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Location of Prop", locationOfProp);
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
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

            Scalar lowerYellow = new Scalar(lowH, lowS, lowV);
            Scalar upperYellow = new Scalar(highH, highS, highV);


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
