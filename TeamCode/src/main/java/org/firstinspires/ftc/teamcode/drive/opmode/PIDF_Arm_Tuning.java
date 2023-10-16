package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "PID ARM TUNING")

public class PIDF_Arm_Tuning extends LinearOpMode {
    private PIDController controller;

    public double power;

    public static double p=0, i = 0, d=0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 0;

    private DcMotorEx arm_motor_left;

    private DcMotorEx arm_motor_right;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        arm_motor_right = hardwareMap.get(DcMotorEx.class,"lift_right");
        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPos = arm_motor_left.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            
            if (pid<0) {
                double power = pid;
            }
            if (pid>=0){
                double power = pid + ff;
            }

            

            arm_motor_left.setPower(power);
            arm_motor_right.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }






}

