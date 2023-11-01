package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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

@Autonomous(name = "RedRight", group = "drive")

public class RedRight extends LinearOpMode {

    private PIDController controller;


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


    public double p = 0.0094, i = 0.05, d=0.0032;

    public double f = 0.3;

    public static int target = 0;

    private final double ticks_in_degrees = 22.4;

    private DcMotorEx lift_left;

    private DcMotorEx lift_right;

    private Servo left_intake;

    private Servo right_intake;

    private Servo right_claw;

    private Servo left_claw;

    TrajectorySequence left1;
    TrajectorySequence left2;

    TrajectorySequence left21;

    TrajectorySequence left3;

    TrajectorySequence left4;

    TrajectorySequence left5;

    TrajectorySequence left6;

    TrajectorySequence left7;

    TrajectorySequence left8;

    TrajectorySequence mid1;

    TrajectorySequence mid2;

    TrajectorySequence mid3;

    TrajectorySequence mid4;

    Trajectory mid5;

    TrajectorySequence mid6;

    TrajectorySequence mid7;

    Trajectory mid8;

    TrajectorySequence mid9;

    Trajectory right1;

    TrajectorySequence right2;

    TrajectorySequence right3;

    TrajectorySequence right4;

    Trajectory right5;

    Trajectory right6;

    Trajectory right7;






    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        lift_right = hardwareMap.get(DcMotorEx.class, "lift_right");

        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");

        lift_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        right_claw = hardwareMap.get(Servo.class, "right claw");
        left_claw = hardwareMap.get(Servo.class, "left claw");

        left_claw.setDirection(Servo.Direction.REVERSE);


        right_intake.setDirection(Servo.Direction.REVERSE);


        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);




        left1 = drive.trajectorySequenceBuilder(startPose)
                .back(33)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 5;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 6;
                    }
                })
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    left_intake.setPosition(1);
//                    right_intake.setPosition(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    right_claw.setPosition(0.1);
//                    left_claw.setPosition(0.1);
//                    target = 100;
//                })

                .UNSTABLE_addTemporalMarkerOffset( 1, () -> {
                    drive.followTrajectorySequenceAsync(left2);
                })

                .build();

        left2 = drive.trajectorySequenceBuilder(left1.end())
                .splineTo(new Vector2d(12, -27), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
//                    target = 40;
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    right_claw.setPosition(1);
//                })
                .UNSTABLE_addTemporalMarkerOffset( 1, () -> {
                    drive.followTrajectorySequenceAsync(left21);
                })

                .build();

        left21 = drive.trajectorySequenceBuilder(left2.end())
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    drive.followTrajectorySequenceAsync(left3);
                })
                .build();


        left3 = drive.trajectorySequenceBuilder(left2.end())
                .forward(10)
                .UNSTABLE_addTemporalMarkerOffset( 3, () -> {
                    drive.followTrajectorySequenceAsync(left4);
                })
                .build();

        left4 = drive.trajectorySequenceBuilder(left3.end())
                .setReversed(true)
                .splineTo(new Vector2d(40, -29), Math.toRadians(0))
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 5;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 6;
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset( 3, () -> {
                    drive.followTrajectorySequenceAsync(left5);
                })
                .build();

        left5 = drive.trajectorySequenceBuilder(left4.end())
                .back(8)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 5;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 6;
                    }
                })

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    target = 100;
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    left_claw.setPosition(0.1);
                })
                .UNSTABLE_addTemporalMarkerOffset( 3, () -> {
                    drive.followTrajectorySequenceAsync(left6);
                })


                .build();

        left6 = drive.trajectorySequenceBuilder(left5.end())
                .forward(5)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 5;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 6;
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset( 3, () -> {
                    drive.followTrajectorySequenceAsync(left7);
                })
                .build();

        left7 = drive.trajectorySequenceBuilder(left6.end())
                .strafeLeft(34)
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 5;
                    }
                })
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 6;
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset( 3, () -> {
                    drive.followTrajectorySequence(left6);
                })
                .build();

        left8 = drive.trajectorySequenceBuilder(left7.end())
                .back(15)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    target = -13;
                })


                .build();


        mid1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    target = 100;
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid2);
                })
                .build();

        mid2 = drive.trajectorySequenceBuilder(mid1.end())
                .splineToConstantHeading(new Vector2d(27, -52), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    target = 30;
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid3);
                })
                .build();

        mid3 = drive.trajectorySequenceBuilder(mid2.end())
                .splineTo(new Vector2d(34, -24), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid4);
                })
                .build();

        mid4 = drive.trajectorySequenceBuilder(mid3.end())
                .back(8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    right_claw.setPosition(0.1);
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(mid5);
                })
                .build();

        mid5 = drive.trajectoryBuilder(mid4.end())
                .forward(8)
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid6);
                })
                .build();

        mid6 = drive.trajectorySequenceBuilder(mid5.end())
                .splineTo(new Vector2d(42,-36),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    target = 150;
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid7);
                })
                .build();

        mid7 = drive.trajectorySequenceBuilder(mid6.end())
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    left_claw.setPosition(0.1);
                })
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(mid8);
                })
                .build();

        mid8 = drive.trajectoryBuilder(mid7.end())
                .strafeLeft(22)
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(mid9);
                })
                .build();

        mid9 = drive.trajectorySequenceBuilder(mid8.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    target = -13;
                })
                .back(8)
                .build();

        right1 = drive.trajectoryBuilder(startPose)
                .back(5)
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(right2);
                })

                .build();

        right2 = drive.trajectorySequenceBuilder(right1.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(23,-42),Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(right3);
                })
                .build();

        right3 = drive.trajectorySequenceBuilder(right2.end())
                .forward(9)
                .addDisplacementMarker(() -> {
                    drive.followTrajectorySequenceAsync(right4);
                })
                .build();

        right4 = drive.trajectorySequenceBuilder(right3.end())
                .setReversed(true)
                .splineTo(new Vector2d(40,-42),Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(right5);
                })
                .build();

        right5 = drive.trajectoryBuilder(right4.end())
                .splineToConstantHeading(new Vector2d(48,-42),Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(right6);
                })
                .build();

        right6 = drive.trajectoryBuilder(right5.end())
                .strafeLeft(17)
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(right7);
                })
                .build();

        right7 = drive.trajectoryBuilder(right6.end())
                .back(10)
                .build();



        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        left_claw.setPosition(1);


        sleep(4000);
        getLocationOfProp();


        waitForStart();





        if (locationOfProp == 1) {
            drive.followTrajectorySequenceAsync(left1);
        }
        if (locationOfProp == 2) {
            drive.followTrajectorySequenceAsync(mid1);

        }
        if (locationOfProp == 3) {
            drive.followTrajectoryAsync(right1);
        }

        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int armPos = (lift_right.getCurrentPosition() + lift_left.getCurrentPosition()) / 2;
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double power = pid + ff;

            lift_left.setPower(power);
            lift_right.setPower(power);

            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Location of Prop", locationOfProp);
            telemetry.update();

            drive.update();

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

    public void lift_up() {
        target = 200;
    }



}
