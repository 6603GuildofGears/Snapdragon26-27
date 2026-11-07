package org.firstinspires.ftc.teamcode.pedro.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


import java.util.List;


/*
 * Gamepad 1 A: toggle Limelight aim-lock. While on, the robot turns itself to stay pointed
 * at the middle of a BIOBUZZ CELL's 4 AprilTags (IDs 30-45, 4 per CELL) while you drive.
 * Moving the right stick overrides it; let go and it re-locks onto the most centered CELL.
 * Hardware config name: "limelight". Driving works without it; aim-lock is just disabled.
 */
@Disabled
@TeleOp(name = "Snapdragon Red-field")
public class Snapdragon_Red extends LinearOpMode {

    private static final double TICKS_PER_REV = 28.0;

    // ---- Aim-lock tuning ----
    static final int    LIMELIGHT_PIPELINE = 0;   // pipeline set up for AprilTags
    static final double AIM_GAIN           = 0.02; // turn power per degree of error
    static final double AIM_MAX_TURN       = 0.5;
    static final double AIM_DEADBAND       = 1.0;  // degrees
    static final double LOST_TIMEOUT_SEC   = 1.0;  // then switch to the most centered CELL
    static final double OVERRIDE_DEADBAND  = 0.1;  // right stick past this = driver override

    // Turn needed per unit of strafe: strafe * STRAFE_TURN_INCHES / distance
    static final double STRAFE_TURN_INCHES = 11.0;
    static final double DEFAULT_DISTANCE_IN = 60.0;
    static final double MIN_DISTANCE_IN     = 18.0;

    static final int FIRST_TAG_ID = 30;
    static final int LAST_TAG_ID  = 37;

    static final double FLOWER_BAR_DOWN = 0.435;
    static final double FLOWER_BAR_UP   = 0.35;

    static final double DRIVE_SPEED = 0.45;
    // Measured peak rpm (FL 5186, BL 5186, FR 5314, BR 5229) matched to the slowest motor
    static final double DRIVE_SCALE_FL = 1.0;
    static final double DRIVE_SCALE_BL = 1.0;
    static final double DRIVE_SCALE_FR = 0.976;
    static final double DRIVE_SCALE_BR = 0.992;
    static final double SHOOTER_RPM = 2500.0;

    // Flywheel encoder counts negative when driven forward, which makes the hub's velocity loop
    // run away to full power. So it runs open loop: power = FLYWHEEL_KF * target ticks/s.
    static final boolean FLYWHEEL_ENCODER_INVERTED = true;
    static final double FLYWHEEL_FREE_SPEED_RPM = 5200.0;
    static final double FLYWHEEL_KF = 1.0 / (FLYWHEEL_FREE_SPEED_RPM * TICKS_PER_REV / 60.0);
    static final double FLYWHEEL_RPM_OFFSET = 550.0; // measured: flywheel runs under the commanded rpm, so command extra
    static final double SHOOTER_READY_MARGIN_RPM = 200.0; // sequence starts once the measured rpm reaches target - this
    static final double SHOOTER_RPM_STEP = 25.0;           // dpad rpm adjustment
    // Extra rpm ramped in once at speed to make up for bogging down; 0 = off (the burst sequence doesn't use it)
    static final double SHOOTER_LOAD_BOOST_RPM = 0.0;
    static final double SHOOTER_LOAD_BOOST_RAMP_SEC = 2.5;
    static final double SHOOTER_AUTO_OFF_SEC = 15.0; // safety: shooter turns itself off this long after it was started
    static final double BLOCKER_OPEN = 0.23;
    static final double BLOCKER_CLOSED = 0.204;
    static final double BLOCKER_OPEN_WAIT_SEC = 0.25; // make sure it is fully open before feeding
    static final double SHOOTER_SPINUP_TIMEOUT_SEC = 2.0; // start anyway if the flywheel never reaches the ready rpm

    // Burst feed sequence: short bursts with gaps, then one long burst
    static final double BURST_INTAKE_POWER = 0.5;
    static final double BURST_SEC = 0.2;
    static final double BURST_GAP_SEC = 0.5;
    static final int SHORT_BURSTS = 3;
    static final double LONG_BURST_SEC = 3.0;

    // Reverse pulse on stick release to cancel momentum
    static final double BRAKE_PULSE_POWER = 0.05;
    static final double BRAKE_PULSE_SEC = 0.25;
    static final double BRAKE_STOPPED_IN_PER_SEC = 2.0; // end the pulse early below this speed
    static final int CELL_COUNT   = 4;

    private int lockedCell = -1; // -1 = none yet
    private final ElapsedTime lostTimer = new ElapsedTime();
    private double targetTx;
    private int targetTagCount;
    private double targetDistance = DEFAULT_DISTANCE_IN;

    private enum ShooterState { OFF, SPINNING_UP, OPENING_BLOCKER, BURST, GAP, LONG_BURST }

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Other motors and servos
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        Servo flowerBar = hardwareMap.get(Servo.class, "flowerBar");

        // Pinpoint
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.REVERSE);
        flowerBar.setDirection(Servo.Direction.REVERSE);

        // Motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Brake when there is no drive input so the robot doesn't coast
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // blocker.setPosition(0.35);
        

        // Limelight (optional)
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        boolean cameraReady = limelight != null;
        if (cameraReady) {
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
        }

        telemetry.addLine("Pinpoint Field Drive Ready");
        telemetry.addData("Limelight", cameraReady ? "ready - press A (gamepad 1) to aim-lock" : "not configured - aim-lock disabled");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        boolean aimOn = false;
        boolean aWasPressed = false;
        boolean bWasPressed = false;
        boolean rbWasPressed = false;
        boolean flowerBarUp = true;
        ShooterState shooterState = ShooterState.OFF;
        ElapsedTime blockerTimer = new ElapsedTime();
        ElapsedTime boostTimer = new ElapsedTime();
        double shooterRpm = SHOOTER_RPM;
        int shortBurstsDone = 0;
        boolean rpmUpWasPressed = false;
        boolean rpmDownWasPressed = false;
        ElapsedTime shooterTimer = new ElapsedTime();
        ElapsedTime brakeTimer = new ElapsedTime();
        double[] lastPowers = new double[4]; // FL, BL, FR, BR
        boolean wasMoving = false;
        boolean braking = false;

        while (opModeIsActive()) {
            pinpoint.update();

            // Driver controls
            double y = -gamepad1.left_stick_y * DRIVE_SPEED;
            double x = gamepad1.left_stick_x * DRIVE_SPEED;
            double rx = gamepad1.right_stick_x * DRIVE_SPEED;

            // Dpad drives inverse robot-relative: up = back, down = forward, left = strafe right, right = strafe left
            if (gamepad1.dpad_up) {
                y = -DRIVE_SPEED;
            } else if (gamepad1.dpad_down) {
                y = DRIVE_SPEED;
            }
            if (gamepad1.dpad_left) {
                x = DRIVE_SPEED;
            } else if (gamepad1.dpad_right) {
                x = -DRIVE_SPEED;
            }

            // Bumpers strafe: right bumper = left, left bumper = right
            if (gamepad1.right_bumper) {
                x = -DRIVE_SPEED;
            } else if (gamepad1.left_bumper) {
                x = DRIVE_SPEED;
            }

            // Reset field heading
            if (gamepad1.y) {
                pinpoint.resetPosAndIMU();
            }

            // Pinpoint heading
            double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);

            // Field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Strafing compensation
            rotX *= 1.1;

            // Aim-lock toggle (only on the press, not while held)
            boolean aPressed = gamepad1.a;
            if (aPressed && !aWasPressed && cameraReady) {
                aimOn = !aimOn;
                lockedCell = -1;
                lostTimer.reset();
            }
            aWasPressed = aPressed;

            String aimStatus = cameraReady ? "OFF (press A)" : "no Limelight";
            if (aimOn) {
                if (Math.abs(gamepad1.right_stick_x) > OVERRIDE_DEADBAND) {
                    // Driver override; re-lock on whatever is closest to the crosshair after release
                    lockedCell = -1;
                    aimStatus = "OVERRIDE";
                } else {
                    // Strafe right -> the CELL slides left -> turn left (negative).
                    // rotX is the robot-relative strafe after the field-centric rotation.
                    double strafeTurn = -rotX * STRAFE_TURN_INCHES / Math.max(targetDistance, MIN_DISTANCE_IN);

                    int cell = findTarget(limelight.getLatestResult());
                    if (cell >= 0) {
                        lockedCell = cell;
                        lostTimer.reset();
                        double correction = Math.abs(targetTx) < AIM_DEADBAND ? 0
                                : Range.clip(targetTx * AIM_GAIN, -AIM_MAX_TURN, AIM_MAX_TURN);
                        rx = strafeTurn + correction;
                        aimStatus = String.format("LOCKED cell %d (%.1f deg off, %d/4 tags, %.0f in)",
                                lockedCell, targetTx, targetTagCount, targetDistance);
                    } else if (lockedCell >= 0 && lostTimer.seconds() < LOST_TIMEOUT_SEC) {
                        rx = strafeTurn;
                        aimStatus = "LOCKED (tags blinked out)";
                    } else {
                        aimStatus = "SEARCHING";
                    }
                }
            }

            // Mecanum motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * DRIVE_SCALE_FL;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * DRIVE_SCALE_BL;
            double frontRightPower = ((rotY - rotX - rx) / denominator) * DRIVE_SCALE_FR;
            double backRightPower = ((rotY + rotX - rx) / denominator) * DRIVE_SCALE_BR;

            // Short reverse pulse when the sticks are released, to cancel momentum
            boolean hasInput = Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx) > 0.01;
            if (hasInput) {
                wasMoving = true;
                braking = false;
                lastPowers[0] = frontLeftPower;
                lastPowers[1] = backLeftPower;
                lastPowers[2] = frontRightPower;
                lastPowers[3] = backRightPower;
            } else {
                if (wasMoving) {
                    wasMoving = false;
                    braking = true;
                    brakeTimer.reset();
                }
                double speed = Math.hypot(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
                if (braking && (brakeTimer.seconds() >= BRAKE_PULSE_SEC || speed < BRAKE_STOPPED_IN_PER_SEC)) {
                    braking = false;
                }
                if (braking) {
                    frontLeftPower = brakePower(lastPowers[0]);
                    backLeftPower = brakePower(lastPowers[1]);
                    frontRightPower = brakePower(lastPowers[2]);
                    backRightPower = brakePower(lastPowers[3]);
                }
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Dpad up/down adjusts the shooter rpm
            boolean rpmUp = gamepad2.dpad_up;
            boolean rpmDown = gamepad2.dpad_down;
            if (rpmUp && !rpmUpWasPressed) shooterRpm += SHOOTER_RPM_STEP;
            if (rpmDown && !rpmDownWasPressed) shooterRpm -= SHOOTER_RPM_STEP;
            rpmUpWasPressed = rpmUp;
            rpmDownWasPressed = rpmDown;
            shooterRpm = Range.clip(shooterRpm, 0, 6000.0);

            // Shooter sequence: right bumper click starts it, click again stops it
            boolean rbPressed = gamepad2.right_bumper;
            if (rbPressed && !rbWasPressed) {
                shooterState = shooterState == ShooterState.OFF ? ShooterState.SPINNING_UP : ShooterState.OFF;
                shooterTimer.reset();
                // Aim-lock follows the shooter: on when shooting starts, off when it stops
                if (cameraReady) {
                    aimOn = shooterState != ShooterState.OFF;
                    lockedCell = -1;
                    lostTimer.reset();
                }
                blockerTimer.reset();
            }
            rbWasPressed = rbPressed;
            if (shooterState != ShooterState.OFF && shooterTimer.seconds() >= SHOOTER_AUTO_OFF_SEC) {
                shooterState = ShooterState.OFF;
                if (cameraReady) {
                    aimOn = false;
                    lockedCell = -1;
                }
            }

            double flywheelRpm = flywheel.getVelocity() * (FLYWHEEL_ENCODER_INVERTED ? -1 : 1) * 60.0 / TICKS_PER_REV;
            if (shooterState == ShooterState.SPINNING_UP
                    && (flywheelRpm >= shooterRpm - SHOOTER_READY_MARGIN_RPM
                        || blockerTimer.seconds() >= SHOOTER_SPINUP_TIMEOUT_SEC)) {
                shooterState = ShooterState.OPENING_BLOCKER;
                boostTimer.reset();
                blockerTimer.reset();
            } else if (shooterState == ShooterState.OPENING_BLOCKER && blockerTimer.seconds() >= BLOCKER_OPEN_WAIT_SEC) {
                shooterState = ShooterState.BURST;
                shortBurstsDone = 0;
                blockerTimer.reset();
            } else if (shooterState == ShooterState.BURST && blockerTimer.seconds() >= BURST_SEC) {
                shortBurstsDone++;
                shooterState = ShooterState.GAP;
                blockerTimer.reset();
            } else if (shooterState == ShooterState.GAP && blockerTimer.seconds() >= BURST_GAP_SEC) {
                shooterState = shortBurstsDone >= SHORT_BURSTS ? ShooterState.LONG_BURST : ShooterState.BURST;
                blockerTimer.reset();
            } else if (shooterState == ShooterState.LONG_BURST && blockerTimer.seconds() >= LONG_BURST_SEC) {
                shooterState = ShooterState.OFF;
                if (cameraReady) {
                    aimOn = false;
                    lockedCell = -1;
                }
            }

            // Intake: bursts feed the shooter, otherwise it stays off while the sequence runs
            if (shooterState == ShooterState.BURST || shooterState == ShooterState.LONG_BURST) {
                intake.setPower(BURST_INTAKE_POWER);
            } else if (shooterState != ShooterState.OFF) {
                intake.setPower(0);
            } else if (gamepad1.right_trigger > 0.1 || gamepad2.left_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                intake.setPower(-0.25);
            } else {
                intake.setPower(0);
            }

            // //Flywheel
            double rpm = 2500.0;
            if (shooterState != ShooterState.OFF) {
                double loadBoost = shooterState != ShooterState.SPINNING_UP
                        ? SHOOTER_LOAD_BOOST_RPM * Math.min(1.0, boostTimer.seconds() / SHOOTER_LOAD_BOOST_RAMP_SEC) : 0;
                flywheel.setPower(flywheelPower(shooterRpm + FLYWHEEL_RPM_OFFSET + loadBoost));
            } else if (gamepad2.right_trigger > 0.1) {
                flywheel.setPower(-flywheelPower(rpm));
            } else {
                flywheel.setPower(0);
        
            }


            // // Blocker
            boolean blockerOpen = gamepad2.a
                    || shooterState == ShooterState.OPENING_BLOCKER
                    || shooterState == ShooterState.BURST
                    || shooterState == ShooterState.GAP
                    || shooterState == ShooterState.LONG_BURST;
            blocker.setPosition(blockerOpen ? BLOCKER_OPEN : BLOCKER_CLOSED);

            // Flower Bar
            boolean bPressed = gamepad2.b;
            if (bPressed && !bWasPressed) {
                flowerBarUp = !flowerBarUp;
            }
            bWasPressed = bPressed;
            flowerBar.setPosition(flowerBarUp ? FLOWER_BAR_UP : FLOWER_BAR_DOWN);
            

            // Telemetry
            telemetry.addData("Aim", aimStatus);
            telemetry.addData("Heading", "%.2f degrees", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint X", "%.2f", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y", "%.2f", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            // telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Shooter", shooterState);
            telemetry.addData("Flywheel RPM", "%.0f (target %.0f, ready at %.0f)", flywheelRpm, shooterRpm, shooterRpm - SHOOTER_READY_MARGIN_RPM);
            telemetry.addData("Flywheel raw", "%.0f ticks/s (target %.0f) | power %.2f", flywheelRpm * TICKS_PER_REV / 60.0, getTickSpeed(shooterRpm), flywheel.getPower());
            telemetry.addData("Flower bar", flowerBarUp ? "UP" : "DOWN");
            telemetry.update();
        }

        if (limelight != null) limelight.stop();
    }

    /**
     * Groups visible CELL tags by CELL and picks one to aim at: the locked CELL if visible,
     * otherwise the one closest to straight ahead. Sets targetTx / targetTagCount / targetDistance.
     * Returns the CELL index, or -1 if none.
     */
    private int findTarget(LLResult result) {
        if (result == null || !result.isValid()) return -1;

        double[] txSum = new double[CELL_COUNT];
        int[] count = new int[CELL_COUNT];
        double[] distSum = new double[CELL_COUNT];
        int[] distCount = new int[CELL_COUNT];

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < FIRST_TAG_ID || id > LAST_TAG_ID) continue;
            int cell = (id - FIRST_TAG_ID) / 4;
            txSum[cell] += tag.getTargetXDegrees();
            count[cell]++;
            double dist = distanceInches(tag);
            if (!Double.isNaN(dist)) {
                distSum[cell] += dist;
                distCount[cell]++;
            }
        }

        int best = -1;
        for (int c = 0; c < CELL_COUNT; c++) {
            if (count[c] == 0) continue;
            if (c == lockedCell) { best = c; break; }
            if (best == -1 || Math.abs(txSum[c] / count[c]) < Math.abs(txSum[best] / count[best])) best = c;
        }
        if (best == -1) return -1;

        // Don't jump to a different CELL just because ours flickered out for a moment
        if (lockedCell >= 0 && best != lockedCell && lostTimer.seconds() < LOST_TIMEOUT_SEC) return -1;

        targetTx = txSum[best] / count[best];
        targetTagCount = count[best];
        if (distCount[best] > 0) targetDistance = distSum[best] / distCount[best];
        return best;
    }

    /** Camera-to-tag distance in inches, or NaN if the Limelight isn't giving 3D poses. */
    private static double distanceInches(LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        double dist = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        return dist < 0.1 ? Double.NaN : dist;
    }

    /** Opposite-direction pulse for one wheel, never stronger than the power it was running at. */
    private static double brakePower(double lastPower) {
        return -Math.signum(lastPower) * Math.min(BRAKE_PULSE_POWER, Math.abs(lastPower));
    }

    /** Open-loop flywheel power for a commanded rpm. */
    private static double flywheelPower(double rpm) {
        return Range.clip(FLYWHEEL_KF * rpm * TICKS_PER_REV / 60.0, 0, 1);
    }

    public double getTickSpeed(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }
}