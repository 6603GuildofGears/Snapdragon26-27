package org.firstinspires.ftc.teamcode.pedro.TeleOp.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Flywheel AutoTuner (6603)", group = "Tuner")
public class Flywheel_AutoTuner extends LinearOpMode {

    private static final String FLYWHEEL_NAME = "flywheel";
    private static final double TICKS_PER_REV = 28.0;
    private static final boolean FLYWHEEL_ENCODER_INVERTED = true;

    private static final double BLOCKER_CLOSED = 0.204;
    private static final double BLOCKER_OPEN = 0.23;

    private double tunedKs = 0.0;
    private double tunedKv = 0.0;
    private double tunedKp = 0.0;
    private double tunedKd = 0.0;
    private double tunedKi = 0.0;

    // Velocity from the hub is quantized/noisy, so control runs on a low-passed RPM
    private static final double RPM_FILTER_TAU_SEC = 0.04;
    private static final double MAX_INTEGRAL_POWER = 0.12;
    private static final double INTEGRATE_WINDOW_RPM = 150.0;
    private double filteredRpm = 0.0;
    private double rpmSlope = 0.0; // RPM/s of the filtered signal
    private double integral = 0.0;

    private DcMotorEx flywheel;
    private DcMotor intake;
    private Servo blocker;
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.tryGet(DcMotor.class, "intake");
        blocker = hardwareMap.tryGet(Servo.class, "blocker");
        if (blocker != null) {
            blocker.setDirection(Servo.Direction.REVERSE);
            blocker.setPosition(BLOCKER_CLOSED);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("=== FTC 6603 FLYWHEEL TUNER & SHOT ANALYZER ===");
        telemetry.addLine("1. Keep shooter EMPTY for first 15s after pressing PLAY.");
        telemetry.addLine("2. After 15s, press [A] to open blocker & run slow intake feed.");
        telemetry.addLine("3. Feed game pieces to measure RPM drop & recovery time in ms!");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        double bestScore;

        // =================================================================
        // STEP 1: 15-Second Automatic Baseline (Empty Shooter)
        // =================================================================
        double rampPower = 0.0;
        while (opModeIsActive() && getFlywheelRpm() < 120.0 &&         rampPower < 0.5) {
            rampPower += 0.002;
            setNormalizedPower(rampPower);
            sleep(25);
            telemetry.addData("Auto-Calibrating (1/3)", "Finding Breakaway Friction (kS)...");
                    telemetry.addData("Ramp power", "%.3f", rampPower);
                    telemetry.addData("Measured RPM", "%.1f (raw %.0f ticks/s)", getFlywheelRpm(), flywheel.getVelocity());
            telemetry.update();
        }
        if (getFlywheelRpm() < 120.0) {
            abortTuner("No flywheel encoder signal: RPM stayed at " + String.format("%.1f", getFlywheelRpm())
                    + " even at " + String.format("%.0f", rampPower * 100) + "% power. "
                    + "Check the encoder cable/port for '" + FLYWHEEL_NAME + "' and TICKS_PER_REV.");
            return;
        }
        tunedKs = rampPower * 0.85;

        double pLow = 0.45;
        double rpmLow = measureSteadyStateRpm(pLow, 3.5, "Auto-Calibrating (2A/3): Low Speed (45%)");

        double pHigh = 0.80;
        double rpmHigh = measureSteadyStateRpm(pHigh, 3.5, "Auto-Calibrating (2B/3): High Speed (80%)");

        if (rpmHigh > rpmLow + 100) {
            tunedKv = (pHigh - pLow) / (rpmHigh - rpmLow);
            tunedKs = Math.max(0.0, pLow - (tunedKv * rpmLow));
        } else {
            abortTuner(String.format("Calibration failed: %.0f RPM at 45%% vs %.0f RPM at 80%%. "
                    + "RPM did not rise with power.", rpmLow, rpmHigh));
            return;
        }

        double baseRpm = 1800.0;
        double targetRpm = 2400.0;
        measureSteadyStateRpm(tunedKs + (tunedKv * baseRpm), 2.5, "Auto-Calibrating (3A/3): Pre-settling at 1800 RPM");

        // Search kP by running real 1800 -> 2400 step responses and scoring the tracking error.
        // Coarse pass over a wide range, then a fine pass around the best value.
        double maxHeadroom = Math.max(0.1, 1.0 - (tunedKs + (tunedKv * targetRpm)));
        double kpBase = maxHeadroom / 220.0;
        double[] coarse = {0.15, 0.3, 0.6, 1.0, 1.6};
        double bestKp = kpBase * 0.6;
        bestScore = Double.MAX_VALUE;
        int trial = 0;
        int totalTrials = coarse.length + 2;
        for (double mult : coarse) {
            double kp = kpBase * mult;
            double score = runStepTrial(kp, baseRpm, targetRpm, ++trial, totalTrials);
            if (score < bestScore) { bestScore = score; bestKp = kp; }
        }
        double coarseBest = bestKp;
        for (double mult : new double[]{0.7, 1.4}) {
            double kp = coarseBest * mult;
            double score = runStepTrial(kp, baseRpm, targetRpm, ++trial, totalTrials);
            if (score < bestScore) { bestScore = score; bestKp = kp; }
        }
        if (!opModeIsActive()) return;
        applyGains(bestKp);

        // =================================================================
        // STEP 2: Interactive Slow-Feed & Shot Recovery Stopwatch
        // =================================================================
        double testTargetRpm = 2400.0;
        double slowIntakePower = 0.5; // Default slow intake speed
        boolean shooterRunning = true;
        boolean slowFeedActive = false; // Toggled with A: opens blocker + runs slow intake

        boolean lastB = false, lastA = false, lastUp = false, lastDown = false;
        boolean lastLb = false, lastRb = false;
        double filteredSteadyRpm = testTargetRpm;
        filteredRpm = getFlywheelRpm();

        // Shot Recovery Analyzer State Machine
        boolean inShotRecovery = false;
        ElapsedTime shotTimer = new ElapsedTime();
        double minRpmDuringShot = testTargetRpm;
        double preShotSteadyRpm = testTargetRpm;
        double lastShotDropRpm = 0.0;
        double lastRecoveryTimeMs = 0.0;
        double postShotPeakRpm = 0.0;
        int shotsDetected = 0;

        ElapsedTime loopTimer = new ElapsedTime();
        double stalledSec = 0.0;

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt > 0.5) dt = 0.02;
            dt = Math.max(dt, 0.004);

            // --- GAMEPAD 1 CONTROLS ---
            // [B] Toggle Flywheel ON/OFF
            if (gamepad1.b && !lastB) shooterRunning = !shooterRunning;
            lastB = gamepad1.b;

            // [A] Toggle Slow-Feed Mode (Blocker Open + Continuous Slow Intake)
            if (gamepad1.a && !lastA) slowFeedActive = !slowFeedActive;
            lastA = gamepad1.a;

            // [D-Pad Up/Down] Target RPM +/- 25
            if (gamepad1.dpad_up && !lastUp) testTargetRpm += 25;
            lastUp = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !lastDown) testTargetRpm -= 25;
            lastDown = gamepad1.dpad_down;

            // [D-Pad Left/Right] Nudge kP Down/Up for tighter recovery (kD/kI follow kP)
            if (gamepad1.dpad_right) applyGains(tunedKp + 0.000005);
            if (gamepad1.dpad_left) applyGains(Math.max(0.0, tunedKp - 0.000005));

            // [Left/Right Bumpers] Nudge Slow Intake Speed +/- 0.05
            if (gamepad1.right_bumper && !lastRb) slowIntakePower = Range.clip(slowIntakePower + 0.05, 0.1, 1.0);
            lastRb = gamepad1.right_bumper;
            if (gamepad1.left_bumper && !lastLb) slowIntakePower = Range.clip(slowIntakePower - 0.05, 0.1, 1.0);
            lastLb = gamepad1.left_bumper;

            // Actuate Blocker & Intake
            if (blocker != null) {
                blocker.setPosition(slowFeedActive ? BLOCKER_OPEN : BLOCKER_CLOSED);
            }
            if (intake != null) {
                intake.setPower(slowFeedActive ? slowIntakePower : 0.0);
            }

            // --- CLOSED-LOOP FLYWHEEL CONTROL ---
            double currentRpm = updateRpmFilter(getFlywheelRpm(), dt);
            double error = testTargetRpm - currentRpm;

            // Track steady-state average when unloaded and near target
            if (!inShotRecovery && Math.abs(error) < 90.0) {
                filteredSteadyRpm = (0.92 * filteredSteadyRpm) + (0.08 * currentRpm);
            }

            // --- AUTOMATIC SHOT DROP & RECOVERY STOPWATCH ---
            // Trigger shot detection when RPM drops more than 100 RPM below target
            if (shooterRunning && !inShotRecovery && error > 100.0 && filteredSteadyRpm > (testTargetRpm - 80.0)) {
                inShotRecovery = true;
                shotTimer.reset();
                minRpmDuringShot = currentRpm;
                preShotSteadyRpm = filteredSteadyRpm;
                postShotPeakRpm = currentRpm;
            }

            if (inShotRecovery) {
                if (currentRpm < minRpmDuringShot) {
                    minRpmDuringShot = currentRpm;
                }
                // Declare recovered once RPM climbs back within 45 RPM of target
                if (error <= 45.0) {
                    lastRecoveryTimeMs = shotTimer.milliseconds();
                    lastShotDropRpm = preShotSteadyRpm - minRpmDuringShot;
                    shotsDetected++;
                    inShotRecovery = false;
                } else if (shotTimer.seconds() > 3.0) {
                    // Timeout if jammed
                    inShotRecovery = false;
                }
            } else if (shotsDetected > 0 && currentRpm > postShotPeakRpm) {
                // Track highest RPM after recovery to check for overshoot
                postShotPeakRpm = currentRpm;
            }

            // Watchdog: full-ish power with no measured speed means the encoder reading is bad
            if (shooterRunning && currentRpm < 50.0) {
                stalledSec += dt;
                if (stalledSec > 1.5) {
                    abortTuner("Flywheel RPM read ~0 while powered for 1.5s. Stopped to avoid runaway power.");
                    return;
                }
            } else {
                stalledSec = 0.0;
            }

            if (shooterRunning) {
                setNormalizedPower(controlPower(testTargetRpm, dt));
            } else {
                integral = 0.0;
                flywheel.setPower(0.0);
            }

            // --- DRIVER HUB TELEMETRY ---
            telemetry.addLine("=== 1.STEADY-STATE & SHOT RECOVERY METRICS ===");
            telemetry.addData("Target vs Live RPM", "%.0f  |  %.1f RPM (Err: %.1f)", testTargetRpm, currentRpm, error);
            telemetry.addData("Steady-State Avg", "%.1f RPM", filteredSteadyRpm);
            telemetry.addData("Last Shot Drop", "-%.0f RPM (Min: %.0f RPM)", lastShotDropRpm, minRpmDuringShot);
            telemetry.addData("Last Recovery Time", "%.0f ms (Shots: %d)", lastRecoveryTimeMs, shotsDetected);
            telemetry.addData("Post-Shot Peak", "%.0f RPM (Overshoot: +%.0f)", postShotPeakRpm, Math.max(0, postShotPeakRpm - testTargetRpm));
            telemetry.addLine("=== 2. SLOW-FEED CONTROLS ===");
            telemetry.addData("[A] Slow-Feed Mode", slowFeedActive ? "ACTIVE (Blocker OPEN, Intake ON)" : "OFF (Blocker CLOSED)");
            telemetry.addData("[LB/RB] Intake Speed", "%.2f", slowIntakePower);
            telemetry.addData("[Dpad L/R] Tune kP", "%.7f  |  [Dpad U/D] RPM", tunedKp);
            telemetry.addLine("=== 3. COPY THESE TO SNAPDRAGON TELEOP ===");
            telemetry.addData("KS / KV", "KS=%.6f  |  KV=%.8f", tunedKs, tunedKv);
            telemetry.addData("KP / KD / KI", "KP=%.7f  |  KD=%.7f  |  KI=%.7f", tunedKp, tunedKd, tunedKi);
            telemetry.update();
        }
    }

    private void applyGains(double kp) {
        tunedKp = kp;
        tunedKd = kp * 0.008;
        tunedKi = kp * 2.0;
    }

    private double updateRpmFilter(double rawRpm, double dt) {
        double alpha = dt / (RPM_FILTER_TAU_SEC + dt);
        double prev = filteredRpm;
        filteredRpm += alpha * (rawRpm - filteredRpm);
        rpmSlope = (filteredRpm - prev) / dt;
        return filteredRpm;
    }

    // Feedforward + PD on filtered RPM + clamped integral that only accumulates near the target
    private double controlPower(double targetRpm, double dt) {
        double error = targetRpm - filteredRpm;
        if (Math.abs(error) < INTEGRATE_WINDOW_RPM) {
            integral = Range.clip(integral + tunedKi * error * dt, -MAX_INTEGRAL_POWER, MAX_INTEGRAL_POWER);
        }
        double ff = tunedKs + (tunedKv * targetRpm);
        return ff + (tunedKp * error) - (tunedKd * rpmSlope) + integral;
    }

    // Settles at fromRpm, steps to toRpm, and scores tracking error (lower is better)
    private double runStepTrial(double kp, double fromRpm, double toRpm, int trial, int totalTrials) {
        applyGains(kp);
        integral = 0.0;
        filteredRpm = getFlywheelRpm();

        ElapsedTime t = new ElapsedTime();
        ElapsedTime loop = new ElapsedTime();
        double iae = 0.0;
        double peak = 0.0;
        double settleSec = 1.5;
        double stepSec = 1.5;

        while (opModeIsActive() && t.seconds() < settleSec + stepSec) {
            double dt = Math.max(loop.seconds(), 0.004);
            loop.reset();
            boolean stepping = t.seconds() >= settleSec;
            double target = stepping ? toRpm : fromRpm;
            double rpm = updateRpmFilter(getFlywheelRpm(), dt);
            setNormalizedPower(controlPower(target, dt));
            if (stepping) {
                iae += Math.abs(target - rpm) * dt;
                peak = Math.max(peak, rpm);
            }
            telemetry.addData("Auto-Tuning kP (3/3)", "Trial %d/%d  kP=%.7f", trial, totalTrials, kp);
            telemetry.addData("Phase", stepping ? "Step to target" : "Settling");
            telemetry.addData("Live RPM", "%.0f", rpm);
            telemetry.update();
            sleep(5);
        }
        double overshoot = Math.max(0.0, peak - toRpm);
        return iae + overshoot * 0.5;
    }

    private double getFlywheelRpm() {
        double ticksPerSec = flywheel.getVelocity();
        if (FLYWHEEL_ENCODER_INVERTED) {
            ticksPerSec *= -1.0;
        }
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }

    private void abortTuner(String reason) {
        flywheel.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
        while (opModeIsActive()) {
            flywheel.setPower(0.0);
            telemetry.addLine("=== TUNER ABORTED ===");
            telemetry.addLine(reason);
            telemetry.addData("Raw velocity (ticks/s)", "%.1f", flywheel.getVelocity());
            telemetry.addData("Raw position (ticks)", flywheel.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }
    }

    private void setNormalizedPower(double power12V) {
        double voltage = batteryVoltageSensor.getVoltage();
        double scale = (voltage > 7.0) ? (12.0 / voltage) : 1.0;
        flywheel.setPower(Range.clip(power12V * scale, 0.0, 1.0));
    }

    private double measureSteadyStateRpm(double power12V, double durationSec, String status) {
        ElapsedTime t = new ElapsedTime();
        double sumRpm = 0.0;
        int samples = 0;

        while (opModeIsActive() && t.seconds() < durationSec) {
            setNormalizedPower(power12V);
            double rpm = getFlywheelRpm();
            if (t.seconds() > (durationSec - 0.75)) {
                sumRpm += rpm;
                samples++;
            }
            telemetry.addData("Status", status);
            telemetry.addData("Time Left", "%.1f s", durationSec - t.seconds());
            telemetry.addData("Live RPM", "%.1f", rpm);
            telemetry.update();
            sleep(20);
        }
        return (samples > 0) ? (sumRpm / samples) : getFlywheelRpm();
    }
}