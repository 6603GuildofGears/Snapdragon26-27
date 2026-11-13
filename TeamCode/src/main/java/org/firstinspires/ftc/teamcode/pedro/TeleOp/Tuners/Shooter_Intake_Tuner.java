package org.firstinspires.ftc.teamcode.pedro.TeleOp.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Tuning OpMode for the shooter flywheel, intake and blocker (gamepad 1, no drive).
 *
 *   Dpad up / down    = shooter target rpm +/- 25 (also sets the sequence's ready rpm)
 *   B                 = toggle the shooter on/off
 *   Dpad right / left = intake power +/- 0.05
 *   X                 = toggle the intake on/off
 *   A                 = toggle the blocker open/closed
 *   Right bumper      = run / cancel the burst shoot sequence:
 *                       spin up -> at (target rpm - SEQ_READY_MARGIN_RPM) stop intake and open blocker ->
 *                       3 short intake bursts (with gaps) -> a long 4th burst -> everything off
 */
@TeleOp(name = "Shooter Intake Tuner", group = "Tuning")
public class Shooter_Intake_Tuner extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;

    // Same open-loop flywheel setup as the match TeleOps
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_FREE_SPEED_RPM = 5200.0;
    static final double FLYWHEEL_KF = 1.0 / (FLYWHEEL_FREE_SPEED_RPM * TICKS_PER_REV / 60.0);
    static final double FLYWHEEL_RPM_OFFSET = 550.0;

    static final double START_RPM = 2375.0;
    static final double RPM_STEP = 25.0;
    static final double MAX_RPM = 6000.0;

    static final double START_INTAKE_POWER = 0.25;
    static final double INTAKE_STEP = 0.05;

    static final double BLOCKER_OPEN = 0.23;
    static final double BLOCKER_CLOSED = 0.204;

    // Burst shoot sequence
    static final double SEQ_READY_MARGIN_RPM = 200.0; // sequence starts at (target rpm - this), 2375 - 200 = 2175
    static final double SEQ_BLOCKER_WAIT_SEC = 0.25; // let the blocker open before the first burst
    static final double BURST_INTAKE_POWER = 0.5;
    static final double BURST_SEC = 0.2;
    static final double BURST_GAP_SEC = 0.5;
    static final int SHORT_BURSTS = 3;
    static final double LONG_BURST_SEC = 3.0;

    private enum Sequence { IDLE, SPIN_UP, OPENING, BURST, GAP, LONG_BURST }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotor.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.REVERSE);

        double targetRpm = START_RPM;
        double intakePower = START_INTAKE_POWER;
        boolean shooterOn = false;
        boolean intakeOn = false;
        boolean blockerOpen = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        boolean lastB = false, lastX = false, lastA = false, lastRb = false;
        Sequence sequence = Sequence.IDLE;
        int shortBurstsDone = 0;
        ElapsedTime seqTimer = new ElapsedTime();

        // Sending a position during INIT starts the servo pulses so it holds before Play
        blocker.setPosition(BLOCKER_CLOSED);

        telemetry.addLine("Dpad up/down = shooter rpm +/- 25, B = shooter on/off");
        telemetry.addLine("Dpad right/left = intake power +/- 0.05, X = intake on/off");
        telemetry.addLine("A = blocker open/closed, Right bumper = burst shoot sequence");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean a = gamepad1.a;
            boolean rb = gamepad1.right_bumper;

            if (up && !lastUp) targetRpm += RPM_STEP;
            if (down && !lastDown) targetRpm -= RPM_STEP;
            if (right && !lastRight) intakePower += INTAKE_STEP;
            if (left && !lastLeft) intakePower -= INTAKE_STEP;
            if (b && !lastB) shooterOn = !shooterOn;
            if (x && !lastX) intakeOn = !intakeOn;
            if (a && !lastA) blockerOpen = !blockerOpen;
            lastUp = up;
            lastDown = down;
            lastLeft = left;
            lastRight = right;
            lastB = b;
            lastX = x;
            lastA = a;

            double flywheelRpm = flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1) * 60.0 / TICKS_PER_REV;

            // Right bumper starts the burst sequence, or cancels it and turns everything off
            if (rb && !lastRb) {
                if (sequence == Sequence.IDLE) {
                    sequence = Sequence.SPIN_UP;
                    shortBurstsDone = 0;
                    seqTimer.reset();
                } else {
                    sequence = Sequence.IDLE;
                    shooterOn = false;
                    intakeOn = false;
                    blockerOpen = false;
                }
            }
            lastRb = rb;

            switch (sequence) {
                case SPIN_UP:
                    if (flywheelRpm >= targetRpm - SEQ_READY_MARGIN_RPM) {
                        sequence = Sequence.OPENING;
                        seqTimer.reset();
                    }
                    break;
                case OPENING:
                    if (seqTimer.seconds() >= SEQ_BLOCKER_WAIT_SEC) {
                        sequence = Sequence.BURST;
                        seqTimer.reset();
                    }
                    break;
                case BURST:
                    if (seqTimer.seconds() >= BURST_SEC) {
                        shortBurstsDone++;
                        sequence = Sequence.GAP;
                        seqTimer.reset();
                    }
                    break;
                case GAP:
                    if (seqTimer.seconds() >= BURST_GAP_SEC) {
                        sequence = shortBurstsDone >= SHORT_BURSTS ? Sequence.LONG_BURST : Sequence.BURST;
                        seqTimer.reset();
                    }
                    break;
                case LONG_BURST:
                    if (seqTimer.seconds() >= LONG_BURST_SEC) {
                        sequence = Sequence.IDLE;
                        shooterOn = false;
                        intakeOn = false;
                        blockerOpen = false;
                    }
                    break;
                default:
                    break;
            }

            boolean sequenceActive = sequence != Sequence.IDLE;
            boolean bursting = sequence == Sequence.BURST || sequence == Sequence.LONG_BURST;
            boolean sequenceBlockerOpen = sequenceActive && sequence != Sequence.SPIN_UP;

            targetRpm = Range.clip(targetRpm, 0, MAX_RPM);
            intakePower = Range.clip(intakePower, 0, 1);

            double flywheelPower = shooterOn || sequenceActive
                    ? Range.clip(FLYWHEEL_KF * (targetRpm + FLYWHEEL_RPM_OFFSET) * TICKS_PER_REV / 60.0, 0, 1)
                    : 0;
            flywheel.setPower(flywheelPower);
            if (sequenceActive) {
                intake.setPower(bursting ? BURST_INTAKE_POWER : 0);
            } else {
                intake.setPower(intakeOn ? intakePower : 0);
            }
            blocker.setPosition(sequenceBlockerOpen || (!sequenceActive && blockerOpen) ? BLOCKER_OPEN : BLOCKER_CLOSED);

            telemetry.addData("Sequence", "%s (%.2f s)", sequence, seqTimer.seconds());
            if (sequence == Sequence.BURST) {
                telemetry.addData("Burst", "%d of %d short", shortBurstsDone + 1, SHORT_BURSTS);
            }
            telemetry.addData("Shooter", shooterOn || sequenceActive ? "ON" : "OFF");
            telemetry.addData("Target rpm", "%.0f (commanded %.0f, sequence starts at %.0f)", targetRpm,
                    targetRpm + FLYWHEEL_RPM_OFFSET, targetRpm - SEQ_READY_MARGIN_RPM);
            telemetry.addData("Flywheel rpm", "%.0f | power %.2f", flywheelRpm, flywheelPower);
            telemetry.addData("Intake", intake.getPower() != 0 ? "ON" : "OFF");
            telemetry.addData("Intake power", "%.2f (burst %.2f)", intakePower, BURST_INTAKE_POWER);
            telemetry.addData("Blocker", blockerOpen || sequenceBlockerOpen ? "OPEN" : "CLOSED");
            telemetry.update();
        }

        flywheel.setPower(0);
        intake.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
    }
}
