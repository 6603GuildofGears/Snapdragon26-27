package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Flywheel + blocker + intake, with the same shoot sequence the TeleOps use. */
public class Shooter {
    static final double TICKS_PER_REV = 28.0;
    static final double SHOOTER_RPM = 2400.0;

    // Flywheel encoder counts negative when driven forward, so the hub's own velocity loop can't be used.
    // The flywheel runs closed loop here instead (gains from Flywheel_AutoTuner, in 12V-normalized power):
    // power = KS + KV * target + KP * error - KD * rpm slope + clamped integral
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_KS = 0.026683;
    static final double FLYWHEEL_KV = 0.00023790;
    static final double FLYWHEEL_KP = 0.0004537;
    static final double FLYWHEEL_KD = 0.0000036;
    static final double FLYWHEEL_KI = 0.0009073;
    static final double FLYWHEEL_FILTER_TAU_SEC = 0.04;        // low-pass on the noisy hub velocity
    static final double FLYWHEEL_MAX_INTEGRAL = 0.12;          // integral term power cap
    static final double FLYWHEEL_INTEGRATE_WINDOW_RPM = 150.0; // only integrate this close to target
    static final double READY_MARGIN_RPM = 100.0;    // feed starts once the rpm is at target, up to this far above it (never below)
    static final double SPINUP_TIMEOUT_SEC = 2.0;    // feed anyway if the flywheel never reaches the ready rpm

    static final double BLOCKER_OPEN = 0.23;
    static final double BLOCKER_CLOSED = 0.204;
    static final double BLOCKER_OPEN_WAIT_SEC = 0.25;

    // Feed: one continuous feed with no pauses
    static final double FEED_INTAKE_POWER = 0.5;
    static final double FEED_SEC = 3.0;
    static final double FEED_BOOST_DELAY_SEC = 0.25; // feeding this long before the boost kicks in
    static final double FEED_BOOST_RPM = 200.0;      // extra target rpm while feeding, to offset the drop from fast feeding

    private final DcMotorEx flywheel;
    private final DcMotor intake;
    private final Servo blocker;
    private final VoltageSensor voltageSensor;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean active;   // flywheel loop only drives the motor during a shot
    private boolean feeding;
    private double filteredRpm;
    private double integral;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotor.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.REVERSE);
        blocker.setPosition(BLOCKER_CLOSED);
    }

    /** Spin up (closed loop), open the blocker, feed continuously, then shut everything off. */
    public Command shoot() {
        Command sequence = sequential(
                instant(this::beginShot),
                race(waitUntil(this::atSpeed), waitMs(SPINUP_TIMEOUT_SEC * 1000)),
                instant(() -> blocker.setPosition(BLOCKER_OPEN)),
                waitMs(BLOCKER_OPEN_WAIT_SEC * 1000),
                instant(this::startFeeding),
                waitMs(FEED_SEC * 1000));

        // The flywheel loop runs alongside the whole sequence and ends with it
        return sequential(
                race(infinite(this::controlFlywheel), sequence),
                instant(this::stop));
    }

    /** Runs the intake directly, e.g. 1.0 for full-power intaking. */
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    /** Turns everything off and closes the blocker. Also call this when the OpMode ends. */
    public void stop() {
        active = false;
        feeding = false;
        flywheel.setPower(0);
        intake.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
    }

    private double readRpm() {
        return flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1) * 60.0 / TICKS_PER_REV;
    }

    private void beginShot() {
        filteredRpm = readRpm();
        integral = 0;
        feeding = false;
        loopTimer.reset();
        active = true;
    }

    private void startFeeding() {
        feeding = true;
        feedTimer.reset();
        intake.setPower(FEED_INTAKE_POWER);
    }

    /** Feedforward + PD on the filtered rpm + clamped integral, run every scheduler cycle during a shot. */
    private void controlFlywheel() {
        if (!active) return;

        double dt = Math.max(loopTimer.seconds(), 0.004);
        if (dt > 0.5) dt = 0.02;
        loopTimer.reset();

        double previousRpm = filteredRpm;
        filteredRpm += (dt / (FLYWHEEL_FILTER_TAU_SEC + dt)) * (readRpm() - filteredRpm);
        double rpmSlope = (filteredRpm - previousRpm) / dt;

        // After the first moments of feeding, raise the target to offset the rpm lost to fast feeding
        double boost = feeding && feedTimer.seconds() >= FEED_BOOST_DELAY_SEC ? FEED_BOOST_RPM : 0.0;
        double targetRpm = SHOOTER_RPM + boost;
        double error = targetRpm - filteredRpm;
        if (Math.abs(error) < FLYWHEEL_INTEGRATE_WINDOW_RPM) {
            integral = Range.clip(integral + FLYWHEEL_KI * error * dt,
                    -FLYWHEEL_MAX_INTEGRAL, FLYWHEEL_MAX_INTEGRAL);
        }
        double power = FLYWHEEL_KS + FLYWHEEL_KV * targetRpm
                + FLYWHEEL_KP * error - FLYWHEEL_KD * rpmSlope + integral;

        // Gains are tuned at 12V, so scale for the current battery voltage
        double volts = voltageSensor.getVoltage();
        double voltageScale = volts > 7.0 ? 12.0 / volts : 1.0;
        flywheel.setPower(Range.clip(power * voltageScale, 0, 1));
    }

    private boolean atSpeed() {
        return filteredRpm >= SHOOTER_RPM && filteredRpm <= SHOOTER_RPM + READY_MARGIN_RPM;
    }
}
