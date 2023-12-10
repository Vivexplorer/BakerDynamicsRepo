package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueLeft")

public class BlueLeft extends LinearOpMode {

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

    private int RightArmUpPos = 150;
    private int LeftArmUpPos = 150;



    private final double ticks_in_degrees = 22.5;

    private DcMotorEx lift_left;

    private DcMotorEx lift_right;

    private Servo right_intake;
    private Servo left_intake;

    private Servo Basket;




    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Basket = hardwareMap.get(Servo.class, "basket");


        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");


        lift_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorEx.class, "lift_right");





        Pose2d startPose = new Pose2d(12,60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(-11)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(-21)
                .build();
        Trajectory traj2Next = drive.trajectoryBuilder(traj2.end())
                .back(-8)
                .build();
        Trajectory traj2Next2 = drive.trajectoryBuilder(traj2Next.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .forward(-24)
                .build();
        Trajectory GOtoBACKDROP = drive.trajectoryBuilder(traj2Next2.end())
                .strafeRight(-6)
                .build();
        Trajectory GOtoBACKDROP2 = drive.trajectoryBuilder(GOtoBACKDROP.end())
                .forward(-8.5)
                .build();
        Trajectory GOBACK = drive.trajectoryBuilder(GOtoBACKDROP2.end())
                .back(-4)
                .build();
        Trajectory traj2Next3 = drive.trajectoryBuilder(GOBACK.end())
                .strafeLeft(-21)
                .build();
        Trajectory traj2Next4 = drive. trajectoryBuilder(traj2Next3.end())
                .forward(-6)
                .build();

        //

        Trajectory traj3 = drive.trajectoryBuilder(startPose)
                .strafeLeft(-4)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(-31)
                .build();
        Trajectory traj4Next = drive.trajectoryBuilder(traj4.end())
                .back(-11)
                .build();
        Trajectory traj4Next2 = drive.trajectoryBuilder(traj4Next.end().plus(new Pose2d(0,0, Math.toRadians(90))))
                .forward(-29)
                .build();
        Trajectory trajStrafe = drive.trajectoryBuilder(traj4Next2.end())
                .strafeRight(-2)
                .build();
        Trajectory FWmore = drive.trajectoryBuilder(trajStrafe.end())
                .forward(-10)
                .build();
        Trajectory traj4Next3 = drive.trajectoryBuilder(traj4Next2.end())
                .strafeLeft(-24)
                .build();
        Trajectory traj4Next4 = drive.trajectoryBuilder(traj4Next3.end())
                .forward(-10)
                .build();



        Trajectory traj5 = drive.trajectoryBuilder(startPose)
                .forward(-28)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .forward(-4)
                .build();
        Trajectory traj6Next2 = drive.trajectoryBuilder(traj6.end())
                .back(-36)
                .build();
        Trajectory TRaj69 = drive.trajectoryBuilder(traj6Next2.end().plus(new Pose2d(0,0, Math.toRadians(180))))
                .strafeRight(-110)
                .build();
        Trajectory trajfw2 = drive.trajectoryBuilder(TRaj69.end())
                .forward(-13)
                .build();
        Trajectory back3 = drive.trajectoryBuilder(trajfw2.end())
                .back(-7)
                .build();
        Trajectory traj6Next4 = drive.trajectoryBuilder(back3.end())
                .strafeLeft(-35)
                .build();
        Trajectory traj6Next5 = drive.trajectoryBuilder(traj6Next4.end())
                .forward(-13)
                .build();

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setTargetPosition(0);
        lift_right.setTargetPosition(0);

        lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift_left.setPower(0);
        lift_right.setPower(0);








        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        //int target = 100;

        sleep(5000);



        waitForStart();

        getLocationOfProp();







        //lift the arm
        //drop servo





        if (locationOfProp == 1) {
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj2Next);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(traj2Next2);
            drive.followTrajectory(GOtoBACKDROP);


            lift_left.setTargetPosition(135);
            lift_right.setTargetPosition(135);


            lift_left.setPower(0.7);
            lift_right.setPower(0.7);


            sleep(5000);

            drive.followTrajectory(GOtoBACKDROP2);

            sleep(1000);
            openBasket();
            sleep(2000);
            drive.followTrajectory(GOBACK);
            closeBasket();

            sleep(1000);

            lift_left.setTargetPosition(0);
            lift_right.setTargetPosition(0);


            lift_left.setPower(0.7);
            lift_right.setPower(0.7);

            sleep(2000);

            drive.followTrajectory(traj2Next3);
            drive.followTrajectory(traj2Next4);

        }
        if (locationOfProp == 2) {
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj4Next);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(traj4Next2);
            drive.followTrajectory(trajStrafe);
            sleep(1000);

            lift_left.setTargetPosition(135);
            lift_right.setTargetPosition(135);


            lift_left.setPower(0.7);
            lift_right.setPower(0.7);

            sleep(5000);

            drive.followTrajectory(FWmore);

            openBasket();

            sleep(2000);

            lift_left.setTargetPosition(0);
            lift_right.setTargetPosition(0);

            lift_left.setPower(0.7);
            lift_right.setPower(0.7);

            sleep(3000);
            closeBasket();
            drive.followTrajectory(traj4Next3);
            drive.followTrajectory(traj2Next4);

        }
        if (locationOfProp == 3) {
            drive.followTrajectory(traj5);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(traj6);
            drive.followTrajectory(traj6Next2);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(TRaj69);

            lift_left.setTargetPosition(135);
            lift_right.setTargetPosition(135);

            lift_left.setPower(0.7);
            lift_right.setPower(0.7);

            sleep(5000);

            drive.followTrajectory(trajfw2);

            openBasket();

            sleep(2000);

            drive.followTrajectory(back3);

            lift_left.setTargetPosition(0);
            lift_right.setTargetPosition(0);

            lift_left.setPower(0.7);
            lift_right.setPower(0.7);

            sleep(1000);

            closeBasket();

            drive.followTrajectory(traj6Next4);
            drive.followTrajectory(traj6Next5);

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

            Scalar lowerYellow = new Scalar(10 , 100, 100);
            Scalar upperYellow = new Scalar(50 , 255, 255);


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

    private void openBasket()
    {
        Basket.setPosition(0.5);
    }

    private void closeBasket()
    {
        Basket.setPosition(0.1);
    }
    private void intakeDown()
    {
        left_intake.setPosition(0.5);
        right_intake.setPosition(0.5);
    }

    private void intakeUp() {
        left_intake.setPosition(0.7);
        right_intake.setPosition(0.7);
    }

}