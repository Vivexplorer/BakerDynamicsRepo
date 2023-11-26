package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Disabled

@Config

@Autonomous(name = "Individual Encoder Testing")

public class IndividualEncoder extends LinearOpMode {

    double pi = 3.14;


    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front right"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back left"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back right"));





        waitForStart();



        while (opModeIsActive()){
            double leftPos = (WHEEL_RADIUS * 2 *pi * GEAR_RATIO * leftEncoder.getCurrentPosition() / TICKS_PER_REV);
            double rightPos = (WHEEL_RADIUS * 2 *pi * GEAR_RATIO * rightEncoder.getCurrentPosition() / TICKS_PER_REV);
            double frontPos = (WHEEL_RADIUS * 2 *pi * GEAR_RATIO * frontEncoder.getCurrentPosition() / TICKS_PER_REV);
            telemetry.addData("Left Pos", leftPos);
            telemetry.addData("Right Pos", rightPos);
            telemetry.addData("Front Pos", frontPos);
            telemetry.update();

        }

    }
}
