package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous

public class ArmTestEncoder extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx lift_motor_left = hardwareMap.get(DcMotorEx.class, "lift_left");
        DcMotorEx lift_motor_right = hardwareMap.get(DcMotorEx.class, "lift_right");

        lift_motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        waitForStart();

        lift_motor_left.setTargetPosition(150);
        lift_motor_right.setTargetPosition(150);

        lift_motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift_motor_right.setPower(0.5);
        lift_motor_left.setPower(0.5);

        while (opModeIsActive()) {
            telemetry.addData("left encoder", lift_motor_left.getCurrentPosition());
            telemetry.addData("right encoder", lift_motor_right.getCurrentPosition());

        }


    }
}
