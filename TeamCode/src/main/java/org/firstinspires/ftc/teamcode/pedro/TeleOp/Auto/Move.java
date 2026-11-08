package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Move", group = "Autonomous")
public class Move extends LinearOpMode {
    static final double DRIVE_POWER = 0.45;
    static final double DRIVE_SECONDS = 3.5;

    // Same per-motor scaling as the TeleOps so it drives straight
    static final double DRIVE_SCALE_FL = 1.0;
    static final double DRIVE_SCALE_BL = 1.0;
    static final double DRIVE_SCALE_FR = 0.976;
    static final double DRIVE_SCALE_BR = 0.992;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready - drives forward for " + DRIVE_SECONDS + " s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();
        frontLeftMotor.setPower(DRIVE_POWER * DRIVE_SCALE_FL);
        backLeftMotor.setPower(DRIVE_POWER * DRIVE_SCALE_BL);
        frontRightMotor.setPower(DRIVE_POWER * DRIVE_SCALE_FR);
        backRightMotor.setPower(DRIVE_POWER * DRIVE_SCALE_BR);

        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            telemetry.addData("Driving", "%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS);
            telemetry.update();
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
