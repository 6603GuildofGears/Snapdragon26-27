package org.firstinspires.ftc.teamcode.pedro.TeleOp.Tuners;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/*
 * Data collection for a distance -> rpm lookup table (gamepad 1).
 *
 *   B            = burst fire at BASE_RPM + trim: spin up, open the blocker, feed continuously,
 *                  then shut off. Press again to cancel.
 *   Dpad up/down = rpm trim +10 / -10
 *   Dpad left/right = Limelight pipeline -1 / +1 (pick the one set up for AprilTags)
 *   Left bumper  = intake (hold), left trigger = reverse (hold); only while not shooting
 *
 * Park at a distance, trim the rpm until the burst scores, then record the distance and rpm in
 * the data table below. Nothing is interpolated yet.
 * Distance is the average over all visible goal tags. The rpm is locked when the burst starts.
 * Hardware config name: "limelight".
 */
@TeleOp(name = "Shooter Lookup Tuner", group = "Tuning")
public class Shooter_Lookup_Tuner extends LinearOpMode {

    static final double TICKS_PER_REV = 28.0;
    static final int LIMELIGHT_PIPELINE = 0;
    static final int FIRST_TAG_ID = 38;
    static final int LAST_TAG_ID = 45;

    // Collected data (not used by the code): distance in inches -> rpm that scored
    //   dist  rpm
    //   ----  ----
    //   

    static final double BASE_RPM = 2400.0;
    static final double RPM_STEP = 10.0;
    static final double MAX_RPM = 6000.0;

    // Same flywheel model as the match TeleOps: power = KS + KV * rpm + KP * error
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_KS = 0.026683;
    static final double FLYWHEEL_KV = 0.00023790;
    static final double FLYWHEEL_KP = 0.0004537;

    static final double BLOCKER_OPEN = 0.23;
    static final double BLOCKER_CLOSED = 0.204;
    static final double BLOCKER_OPEN_WAIT_SEC = 0.25;
    static final double READY_MARGIN_RPM = 100.0;  // ready once at target, up to this far above it
    static final double SPINUP_TIMEOUT_SEC = 2.5;  // feed anyway if the flywheel never gets there

    static final double FEED_INTAKE_POWER = 0.5;
    static final double FEED_SEC = 3.0;
    static final double FEED_BOOST_DELAY_SEC = 0.25;
    static final double FEED_BOOST_RPM = 200.0;    // extra target while feeding, as in the match TeleOps

    private enum State { IDLE, SPINNING_UP, OPENING_BLOCKER, FEEDING }

    private String tagStatus = "";

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // same as the All-In-One, so 0.5 feeds at the same speed
        blocker.setDirection(Servo.Direction.REVERSE);
        blocker.setPosition(BLOCKER_CLOSED);

        int pipeline = LIMELIGHT_PIPELINE;
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        double rpmTrim = 0;
        double lastDistance = Double.NaN; // keeps the last seen distance when the tags drop out
        double shotRpm = BASE_RPM;
        State state = State.IDLE;
        ElapsedTime stateTimer = new ElapsedTime();
        boolean lastB = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        telemetry.addLine("B = burst fire, Dpad up/down = rpm trim +/- 10, Dpad left/right = Limelight pipeline");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean b = gamepad1.b;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (up && !lastUp) rpmTrim += RPM_STEP;
            if (down && !lastDown) rpmTrim -= RPM_STEP;
            if ((left && !lastLeft) || (right && !lastRight)) {
                pipeline = Range.clip(pipeline + (right && !lastRight ? 1 : -1), 0, 9);
                limelight.pipelineSwitch(pipeline);
            }
            lastLeft = left;
            lastRight = right;
            double distance = goalDistanceInches(limelight.getLatestResult());
            if (!Double.isNaN(distance)) lastDistance = distance;
            double autoRpm = Range.clip(BASE_RPM + rpmTrim, 0, MAX_RPM);

            if (b && !lastB) {
                if (state == State.IDLE) {
                    shotRpm = autoRpm; // locked for the whole burst
                    state = State.SPINNING_UP;
                    stateTimer.reset();
                } else {
                    state = State.IDLE;
                }
            }
            lastB = b;
            lastUp = up;
            lastDown = down;

            double rpm = flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1) * 60.0 / TICKS_PER_REV;

            switch (state) {
                case SPINNING_UP:
                    boolean ready = rpm >= shotRpm && rpm <= shotRpm + READY_MARGIN_RPM;
                    if (ready || stateTimer.seconds() > SPINUP_TIMEOUT_SEC) {
                        blocker.setPosition(BLOCKER_OPEN);
                        state = State.OPENING_BLOCKER;
                        stateTimer.reset();
                    }
                    break;
                case OPENING_BLOCKER:
                    if (stateTimer.seconds() >= BLOCKER_OPEN_WAIT_SEC) {
                        state = State.FEEDING;
                        stateTimer.reset();
                    }
                    break;
                case FEEDING:
                    if (stateTimer.seconds() >= FEED_SEC) state = State.IDLE;
                    break;
                default:
                    break;
            }

            if (state == State.IDLE) {
                flywheel.setPower(0);
                blocker.setPosition(BLOCKER_CLOSED);
                intake.setPower(gamepad1.left_bumper ? 1.0 : gamepad1.left_trigger > 0.1 ? -0.25 : 0);
            } else {
                intake.setPower(state == State.FEEDING ? FEED_INTAKE_POWER : 0);
                boolean boost = state == State.FEEDING && stateTimer.seconds() >= FEED_BOOST_DELAY_SEC;
                double target = shotRpm + (boost ? FEED_BOOST_RPM : 0);
                double power = FLYWHEEL_KS + FLYWHEEL_KV * target + FLYWHEEL_KP * (target - rpm);
                flywheel.setPower(Range.clip(power, 0, 1));
            }

            telemetry.addData("Distance (in)", Double.isNaN(distance)
                    ? (Double.isNaN(lastDistance) ? "no tag yet" : String.format("no tag (last %.1f)", lastDistance))
                    : String.format("%.1f", distance));
            telemetry.addData("Pipeline (requested)", pipeline);
            telemetry.addData("Limelight", "connected=%b running=%b | last update %d ms ago",
                    limelight.isConnected(), limelight.isRunning(), limelight.getTimeSinceLastUpdate());
            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("Limelight status", "pipeline %d | %.0f fps | %.0f C",
                        status.getPipelineIndex(), status.getFps(), status.getTemp());
            }
            telemetry.addData("Tags", tagStatus);
            telemetry.addData("Trim", "%+.0f", rpmTrim);
            telemetry.addData("Shot rpm (base + trim)", "%.0f", autoRpm);
            telemetry.addData("Flywheel rpm", "%.0f", rpm);
            telemetry.addData("State", state);
            telemetry.update();
        }

        flywheel.setPower(0);
        intake.setPower(0);
        blocker.setPosition(BLOCKER_CLOSED);
        limelight.stop();
    }

    /** Average camera-to-tag distance in inches over the visible goal tags, or NaN if none. Sets tagStatus. */
    private double goalDistanceInches(LLResult result) {
        if (result == null) {
            tagStatus = "no result from the Limelight";
            return Double.NaN;
        }

        double sum = 0;
        int count = 0;
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            ids.append(id).append(' ');
            if (id < FIRST_TAG_ID || id > LAST_TAG_ID) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) continue;
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double dist = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (dist < 0.1) continue; // all-zero pose = 3D not enabled on the pipeline
            sum += dist;
            count++;
        }
        if (ids.length() == 0) {
            tagStatus = String.format("pipeline %d returned no tags (valid=%b, %d ms old)",
                    result.getPipelineIndex(), result.isValid(), result.getStaleness());
        } else if (count == 0) {
            tagStatus = "tags seen [" + ids.toString().trim() + "] but none in ID range " + FIRST_TAG_ID + "-" + LAST_TAG_ID
                    + " with a usable 3D pose";
        } else {
            tagStatus = "using " + count + " tag(s) [" + ids.toString().trim() + "]";
        }
        return count == 0 ? Double.NaN : sum / count;
    }
}
