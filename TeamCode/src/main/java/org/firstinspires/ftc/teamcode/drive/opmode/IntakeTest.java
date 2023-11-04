package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="intaketest")

public class IntakeTest extends LinearOpMode {

    private DcMotorEx intake;
    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(1.0);
        }

    }
}
