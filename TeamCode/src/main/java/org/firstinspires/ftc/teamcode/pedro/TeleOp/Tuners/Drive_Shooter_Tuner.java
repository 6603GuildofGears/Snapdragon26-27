package org.firstinspires.ftc.teamcode.pedro.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/*
 * Tuning OpMode for the drive motors and the shooter flywheel (gamepad 1).
 *
 *   X          = toggle all four drive motors at full power (x its scale below)
 *   A          = toggle the shooter flywheel at the target rpm
 *   Dpad up    = target rpm +100      Dpad down = target rpm -100
 *   Back       = reset the peak rpm readings
 *
 * Drive: run with every scale at 1.0, read the PEAK rpm of each motor, then put the
 * suggested scales (telemetry) into DRIVE_SCALE so the strong motors are slowed down to
 * match the weakest one.
 */
@Disabled
@TeleOp(name = "Drive Shooter Tuner", group = "Tuning")
public class Drive_Shooter_Tuner extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0; // drive motors and flywheel

    // Order: frontLeft, backLeft, frontRight, backRight. 1.0 = full power; lower the strong ones.
    // Measured peaks: FL 5186, BL 5186, FR 5314, BR 5229 rpm -> scaled to the slowest (5186)
    static final double[] DRIVE_SCALE = {1.0, 1.0, 0.976, 0.992};

    static final double START_RPM = 1500.0;
    static final double RPM_STEP = 100.0;
    static final double MAX_RPM = 6000.0;

    // The flywheel encoder counts negative when driven forward, so the hub's velocity loop runs
    // away to full power. Run it open loop (feedforward only) and just read the signed velocity.
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_FREE_SPEED_RPM = 5200.0; // measured no-load top speed
    static final double FLYWHEEL_RPM_OFFSET = 550.0; // flywheel runs under the commanded rpm, so command extra
    static final double FLYWHEEL_KF = 1.0 / (FLYWHEEL_FREE_SPEED_RPM * TICKS_PER_REV / 60.0);

    @Override
    public void runOpMode() throws InterruptedException {
        String[] names = {"frontLeftMotor", "backLeftMotor", "frontRightMotor", "backRightMotor"};
        DcMotorEx[] drive = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            drive[i] = hardwareMap.get(DcMotorEx.class, names[i]);
            drive[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // Same directions as the match TeleOps so positive power = forward
        drive[0].setDirection(DcMotor.Direction.REVERSE);
        drive[1].setDirection(DcMotor.Direction.REVERSE);
        drive[2].setDirection(DcMotor.Direction.FORWARD);
        drive[3].setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double[] peakRpm = new double[names.length];
        double targetRpm = START_RPM;
        boolean driveOn = false;
        boolean shooterOn = false;
        boolean lastX = false, lastA = false, lastUp = false, lastDown = false;

        telemetry.addLine("X = drive toggle, A = shooter toggle, Dpad up/down = rpm +/- " + (int) RPM_STEP);
        telemetry.addLine("Back = reset peak rpm");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean x = gamepad1.x;
            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (x && !lastX) driveOn = !driveOn;
            if (a && !lastA) shooterOn = !shooterOn;
            if (up && !lastUp) targetRpm += RPM_STEP;
            if (down && !lastDown) targetRpm -= RPM_STEP;
            targetRpm = Range.clip(targetRpm, 0, MAX_RPM);
            lastX = x;
            lastA = a;
            lastUp = up;
            lastDown = down;

            if (gamepad1.back) {
                java.util.Arrays.fill(peakRpm, 0);
            }

            for (int i = 0; i < drive.length; i++) {
                drive[i].setPower(driveOn ? DRIVE_SCALE[i] : 0);
                peakRpm[i] = Math.max(peakRpm[i], ticksToRpm(drive[i].getVelocity()));
            }

            double flywheelTicks = flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1);
            double targetTicks = shooterOn ? rpmToTicks(targetRpm) : 0;
            double flywheelPower = shooterOn ? Range.clip(FLYWHEEL_KF * (targetTicks + rpmToTicks(FLYWHEEL_RPM_OFFSET)), 0, 1) : 0;
            flywheel.setPower(flywheelPower);

            // Slowest motor that actually moved is the reference for the suggested scales
            double slowest = Double.MAX_VALUE;
            for (double p : peakRpm) {
                if (p > 1) slowest = Math.min(slowest, p);
            }

            telemetry.addData("Drive", driveOn ? "ON (full power)" : "OFF");
            for (int i = 0; i < drive.length; i++) {
                telemetry.addData(names[i],
                        "%.0f rpm | peak %.0f | scale %.2f | suggested %.3f",
                        drive[i].getVelocity() * 60.0 / TICKS_PER_REV, peakRpm[i], DRIVE_SCALE[i],
                        peakRpm[i] > 1 && slowest != Double.MAX_VALUE ? DRIVE_SCALE[i] * slowest / peakRpm[i] : 1.0);
            }
            telemetry.addLine();
            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target rpm", "%.0f", targetRpm);
            telemetry.addData("Flywheel rpm", "%.0f (signed, should be positive)", flywheelTicks * 60.0 / TICKS_PER_REV);
            telemetry.addData("Flywheel ticks/s", "%.0f (target %.0f) | power %.2f",
                    flywheelTicks, targetTicks, flywheelPower);
            telemetry.update();
        }

        for (DcMotorEx motor : drive) {
            motor.setPower(0);
        }
        flywheel.setPower(0);
    }

    private static double ticksToRpm(double ticksPerSec) {
        return Math.abs(ticksPerSec) * 60.0 / TICKS_PER_REV;
    }

    private static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
}
