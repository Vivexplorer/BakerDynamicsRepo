package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled

@Config

@Autonomous(group = "drive")

public class PIDF_Arm_Tuning extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.0144, i = 0.02, d=0.0032;
    public static double f = 0.3;

    public static int target = 0;

    private final double ticks_in_degrees = 22.4;

    private DcMotorEx arm_motor_left;

    private DcMotorEx arm_motor_right;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        controller = new PIDController(p, i, d);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        arm_motor_right = hardwareMap.get(DcMotorEx.class,"lift_right");

        arm_motor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm_motor_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm_motor_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPos = (arm_motor_left.getCurrentPosition() + arm_motor_right.getCurrentPosition()) / 2;
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double power = pid + ff;

            arm_motor_left.setPower(power);
            arm_motor_right.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("rightArmPosition", arm_motor_right.getCurrentPosition());
            telemetry.addData("leftArmPosition", arm_motor_left.getCurrentPosition());
            telemetry.update();
        }
    }






}

