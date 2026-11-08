package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Shooter;

@Autonomous(name = "Move And Shoot", group = "Autonomous")
public class MoveAndShoot extends LinearOpMode {
    static final double DRIVE_POWER = 0.45;
    static final double TURN_POWER = 0.4;

    static final double FIRST_FORWARD_SEC = 0.1;
    static final double TURN_LEFT_SEC = 0.15;
    static final double SECOND_FORWARD_SEC = 4.0;

    // Same per-motor scaling as the TeleOps so it drives straight
    static final double DRIVE_SCALE_FL = 1.0;
    static final double DRIVE_SCALE_BL = 1.0;
    static final double DRIVE_SCALE_FR = 0.976;
    static final double DRIVE_SCALE_BR = 0.992;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Shooter shooter;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Scheduler.reset();
        shooter = new Shooter(hardwareMap);

        telemetry.addLine("Ready - shoot, forward 0.5 s, turn left 0.25 s, forward 4 s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            Scheduler.execute();
        }

        stopDrive();
        shooter.stop();
    }

    private Command autoRoutine() {
        return sequential(
                shooter.shoot(),
                driveFor(DRIVE_POWER, 0, FIRST_FORWARD_SEC),
                driveFor(0, -TURN_POWER, TURN_LEFT_SEC), // negative turn = left, same as the TeleOp aim-lock
                driveFor(DRIVE_POWER, 0, SECOND_FORWARD_SEC)
        );
    }

    /** Drives with the given forward and turn power for the given time, then stops. */
    private Command driveFor(double forward, double turn, double seconds) {
        return sequential(
                instant(() -> setDrive(forward, turn)),
                waitMs(seconds * 1000),
                instant(this::stopDrive)
        );
    }

    private void setDrive(double forward, double turn) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(turn), 1.0);
        frontLeftMotor.setPower(((forward + turn) / denominator) * DRIVE_SCALE_FL);
        backLeftMotor.setPower(((forward + turn) / denominator) * DRIVE_SCALE_BL);
        frontRightMotor.setPower(((forward - turn) / denominator) * DRIVE_SCALE_FR);
        backRightMotor.setPower(((forward - turn) / denominator) * DRIVE_SCALE_BR);
    }

    private void stopDrive() {
        setDrive(0, 0);
    }
}
