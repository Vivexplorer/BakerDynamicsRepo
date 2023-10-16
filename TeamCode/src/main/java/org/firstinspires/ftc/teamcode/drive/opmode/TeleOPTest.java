package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")

public class TeleOPTest extends OpMode {
    private PIDController controller;

    public static double p = 0.02, i = 0, d=0.002;
    public static double f = 0.3;

    public static int target = 0;

    public static double MOTOR_POWERS = 0.6;

    private final double ticks_in_degrees = 22.4;

    private DcMotorEx lift_motor_left;
    private DcMotorEx lift_motor_right;

    SampleMecanumDrive drive ;
    @Override
    public void init() {
         drive = new SampleMecanumDrive(hardwareMap);

        lift_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        lift_motor_right = hardwareMap.get(DcMotorEx.class, "lift_right");

        controller = new PIDController(p, i, d);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        lift_motor_left.setDirection(DcMotorSimple.Direction.REVERSE);





    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);

        int armPos = lift_motor_left.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        lift_motor_right.setPower(power);
        lift_motor_left.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();

        if (gamepad1.a) {
            lift_up();
        }else if (gamepad1.y) {
            lift_neutral();
        }else {
            target = 0;
        }

        if (gamepad1.left_stick_y>=0) {
            drive.setMotorPowers(MOTOR_POWERS,MOTOR_POWERS,MOTOR_POWERS,MOTOR_POWERS);
        }
        if (gamepad1.left_stick_y<0) {
            drive.setMotorPowers(-MOTOR_POWERS,-MOTOR_POWERS,-MOTOR_POWERS,-MOTOR_POWERS);
        }


    }
    public void lift_up() {
        target = 235;
    }
    public void lift_neutral() {
        target = 20;
    }
}
