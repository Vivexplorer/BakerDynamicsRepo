package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;


@TeleOp(group = "drive")

public class TeleOPTest extends OpMode {
    private PIDController controller;

    public double p = 0.0094, i = 0.05, d=0.0032;

    public double f = 0.3;

    public static int target = -10;

    public static double MOTOR_POWERS;

    private final double ticks_in_degrees = 22.4;

    private DcMotorEx lift_motor_left;
    private DcMotorEx lift_motor_right;

    private Servo left_intake;
    private Servo right_intake;

    private DcMotorEx intake;

    private Servo right_claw;

    private Servo left_claw;





    SampleMecanumDrive drive ;

    ElapsedTime elapsedTime = new ElapsedTime();





    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);

        lift_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        lift_motor_right = hardwareMap.get(DcMotorEx.class, "lift_right");

        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");

        right_claw = hardwareMap.get(Servo.class, "left claw");
        left_claw = hardwareMap.get(Servo.class, "right claw");

        controller = new PIDController(p, i, d);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        intake = hardwareMap.get(DcMotorEx.class, "intake");

        lift_motor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift_motor_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift_motor_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        left_claw.setDirection(Servo.Direction.REVERSE);

        right_intake.setDirection(Servo.Direction.REVERSE);



        MOTOR_POWERS = 0.55;

        target = -13;







    }
    @Override
    public void loop() {


        controller.setPID(p, i, d);

        int armPos = (lift_motor_left.getCurrentPosition() + lift_motor_right.getCurrentPosition()) / 2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = pid + ff;

        lift_motor_right.setPower(power);
        lift_motor_left.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("rightArmPosition", lift_motor_right.getCurrentPosition());
        telemetry.addData("leftArmPosition", lift_motor_left.getCurrentPosition());
        telemetry.update();

        if (gamepad2.a) {
            lift_up();
        }else if (gamepad2.y) {
            left_claw.setPosition(0.1);
            right_claw.setPosition(0.1);
            lift_neutral();
        }

        if (gamepad1.x) {
            intake.setPower(-1.0);
        }else if (gamepad1.b) {
            intake.setPower(1);
        }else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_left) {
            intake_up();
        }else if (gamepad1.dpad_right) {
            intake_down();
        }

        if (gamepad2.left_bumper) {
            left_claw.setPosition(0.1);
        }

        if (gamepad2.right_bumper) {
            right_claw.setPosition(0.1);
        }

        if (gamepad2.dpad_down) {
            right_claw.setPosition(1);
            left_claw.setPosition(1);
        }


        if (gamepad2.dpad_up) {
            target -= 10;
        }

        if (gamepad2.options) {
            lift_motor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lift_motor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            lift_motor_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lift_motor_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }




        if (gamepad1.left_stick_y>0) {
            drive.setMotorPowers(MOTOR_POWERS,MOTOR_POWERS,MOTOR_POWERS,MOTOR_POWERS);//forward
        }else if (gamepad1.left_stick_y<0) {
            drive.setMotorPowers(-MOTOR_POWERS,-MOTOR_POWERS,-MOTOR_POWERS,-MOTOR_POWERS);//backward
        }else if (gamepad1.left_stick_x>0) {
            drive.setMotorPowers(-MOTOR_POWERS, -MOTOR_POWERS, MOTOR_POWERS, MOTOR_POWERS);
        }else if(gamepad1.left_stick_x<0) {
            drive.setMotorPowers(MOTOR_POWERS, MOTOR_POWERS, -MOTOR_POWERS, -MOTOR_POWERS);
        }else if (gamepad1.right_stick_x>0) {
            drive.setMotorPowers(-MOTOR_POWERS,MOTOR_POWERS,-MOTOR_POWERS,MOTOR_POWERS);
        }else if (gamepad1.right_stick_x<0) {
            drive.setMotorPowers(MOTOR_POWERS,-MOTOR_POWERS,MOTOR_POWERS,-MOTOR_POWERS);
        }else if (gamepad1.dpad_up) {
            drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
        }else if (gamepad1.dpad_down) {
            drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
        }
        else {
            drive.setMotorPowers(0,0,0,0);
        }


    }
    public void lift_up() {
        target = 245;
    }
    public void lift_neutral() {
        target = -16;
    }

    public void intake_up() {
        left_intake.setPosition(0.5);
        right_intake.setPosition(0.5);
    }
    public void intake_down() {
        left_intake.setPosition(1);
        right_intake.setPosition(1);
    }
}
