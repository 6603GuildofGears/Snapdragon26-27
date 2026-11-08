package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/** Flywheel + blocker + intake, with the same shoot sequence the TeleOps use. */
public class Shooter {
    static final double TICKS_PER_REV = 28.0;
    static final double SHOOTER_RPM = 2400.0;

    // Higher battery voltage makes the open-loop flywheel spin faster, so the target is bumped down
    static final double HIGH_VOLTAGE = 13.0;
    static final double HIGH_VOLTAGE_RPM_DROP = 40.0;     // above HIGH_VOLTAGE, up to VERY_HIGH_VOLTAGE
    static final double VERY_HIGH_VOLTAGE = 13.4;
    static final double VERY_HIGH_VOLTAGE_RPM_DROP = 60.0; // above VERY_HIGH_VOLTAGE

    // Flywheel encoder counts negative when driven forward, so it runs open loop: power = KF * target ticks/s
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_FREE_SPEED_RPM = 5200.0;
    static final double FLYWHEEL_KF = 1.0 / (FLYWHEEL_FREE_SPEED_RPM * TICKS_PER_REV / 60.0);
    static final double FLYWHEEL_RPM_OFFSET = 550.0; // flywheel runs under the commanded rpm, so command extra
    static final double READY_MARGIN_RPM = 200.0;    // feed starts once the measured rpm reaches target - this
    static final double SPINUP_TIMEOUT_SEC = 2.0;    // feed anyway if the flywheel never reaches the ready rpm

    static final double BLOCKER_OPEN = 0.23;
    static final double BLOCKER_CLOSED = 0.204;
    static final double BLOCKER_OPEN_WAIT_SEC = 0.25;

    // Burst feed: short bursts with gaps, then one long burst
    static final double BURST_INTAKE_POWER = 0.5;
    static final double BURST_SEC = 0.2;
    static final double BURST_GAP_SEC = 0.5;
    static final int SHORT_BURSTS = 3;
    static final double LONG_BURST_SEC = 3.0;

    private final DcMotorEx flywheel;
    private final DcMotor intake;
    private final Servo blocker;
    private final VoltageSensor voltageSensor;
    private double targetRpm = SHOOTER_RPM;

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

    /** Spin up, open the blocker, feed the burst sequence, then shut everything off. */
    public Command shoot() {
        Command burst = sequential(
                instant(() -> intake.setPower(BURST_INTAKE_POWER)),
                waitMs(BURST_SEC * 1000),
                instant(() -> intake.setPower(0)),
                waitMs(BURST_GAP_SEC * 1000));

        return sequential(
                instant(this::spinUp),
                race(waitUntil(this::atSpeed), waitMs(SPINUP_TIMEOUT_SEC * 1000)),
                instant(() -> blocker.setPosition(BLOCKER_OPEN)),
                waitMs(BLOCKER_OPEN_WAIT_SEC * 1000),
                repeat(burst, SHORT_BURSTS),
                instant(() -> intake.setPower(BURST_INTAKE_POWER)),
                waitMs(LONG_BURST_SEC * 1000),
                instant(this::stop));
    }

    /** Runs the intake directly, e.g. 1.0 for full-power intaking. */
    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    /** Turns everything off and closes the blocker. Also call this when the OpMode ends. */
    public void stop() {
        flywheel.setPower(0);
        intake.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
    }

    private void spinUp() {
        // Read the battery once per shot so the target doesn't change mid-sequence
        double volts = voltageSensor.getVoltage();
        double drop = volts > VERY_HIGH_VOLTAGE ? VERY_HIGH_VOLTAGE_RPM_DROP
                : volts > HIGH_VOLTAGE ? HIGH_VOLTAGE_RPM_DROP : 0;
        targetRpm = SHOOTER_RPM - drop;
        double targetTicksPerSec = (targetRpm + FLYWHEEL_RPM_OFFSET) * TICKS_PER_REV / 60.0;
        flywheel.setPower(Range.clip(FLYWHEEL_KF * targetTicksPerSec, 0, 1));
    }

    private boolean atSpeed() {
        double rpm = flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1) * 60.0 / TICKS_PER_REV;
        return rpm >= targetRpm - READY_MARGIN_RPM;
    }
}
