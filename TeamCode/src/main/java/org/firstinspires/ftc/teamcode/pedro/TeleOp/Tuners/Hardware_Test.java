package org.firstinspires.ftc.teamcode.pedro.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Hardware test: hold a button to run that motor forward, push a stick to move a servo.
 *
 * Gamepad 1:
 *   A = frontLeftMotor     B = frontRightMotor
 *   X = backLeftMotor      Y = backRightMotor
 *   Right bumper = intake  Left bumper = flywheel
 *   Left stick Y  (up = increase) = blocker position
 *   Dpad up = flowerBar up position, Dpad down = flowerBar down position
 */
@Disabled
@TeleOp(name = "Hardware Test", group = "Test")
public class Hardware_Test extends LinearOpMode {

    static final double MOTOR_POWER = 0.5;
    static final double STICK_DEADBAND = 0.1;
    static final double SERVO_SPEED_PER_SEC = 0.5; // position change per second at full stick
    static final double SERVO_START_POS = 0.204; // blocker start (blocking)
    static final double FLOWER_BAR_DOWN = 0.420;
    static final double FLOWER_BAR_UP   = 0.32;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        Servo blocker = hardwareMap.get(Servo.class, "blocker");
        Servo flowerBar = hardwareMap.get(Servo.class, "flowerBar");

        // Same directions as the TeleOps
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        blocker.setDirection(Servo.Direction.REVERSE);
        flowerBar.setDirection(Servo.Direction.REVERSE);

        double blockerPos = SERVO_START_POS;
        double flowerBarPos = FLOWER_BAR_UP;

        // Sending a position during INIT starts the servo pulses so they hold before Play
        blocker.setPosition(blockerPos);
        flowerBar.setPosition(flowerBarPos);

        telemetry.addLine("Hardware Test ready");
        telemetry.addLine("A/B/X/Y = drive motors, bumpers = intake/flywheel");
        telemetry.addLine("Left stick Y = blocker; Dpad up/down = flowerBar up/down position");
        telemetry.addData("blocker starts at", SERVO_START_POS);
        telemetry.addData("flowerBar starts at (up)", FLOWER_BAR_UP);
        telemetry.addData("flowerBar down position", FLOWER_BAR_DOWN);
        telemetry.addData("Battery (V)", "%.2f", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addLine("Battery near 0-5 V = hub on USB only, servos/motors unpowered");
        telemetry.update();

        waitForStart();
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            frontLeftMotor.setPower(gamepad1.a ? MOTOR_POWER : 0);
            frontRightMotor.setPower(gamepad1.b ? MOTOR_POWER : 0);
            backLeftMotor.setPower(gamepad1.x ? MOTOR_POWER : 0);
            backRightMotor.setPower(gamepad1.y ? MOTOR_POWER : 0);
            intake.setPower(gamepad1.right_bumper ? MOTOR_POWER : 0);
            flywheel.setPower(gamepad1.left_bumper ? MOTOR_POWER : 0);

            blockerPos = stepServo(blockerPos, -gamepad1.left_stick_y, dt);
            if (gamepad1.dpad_up) {
                flowerBarPos = FLOWER_BAR_UP;
            } else if (gamepad1.dpad_down) {
                flowerBarPos = FLOWER_BAR_DOWN;
            }
            blocker.setPosition(blockerPos);
            flowerBar.setPosition(flowerBarPos);

            telemetry.addLine("--- Motors (power | encoder ticks) ---");
            motorLine("A frontLeftMotor", frontLeftMotor);
            motorLine("B frontRightMotor", frontRightMotor);
            motorLine("X backLeftMotor", backLeftMotor);
            motorLine("Y backRightMotor", backRightMotor);
            motorLine("RB intake", intake);
            motorLine("LB flywheel", flywheel);
            telemetry.addData("flywheel velocity (ticks/s)", "%.0f", flywheel.getVelocity());
            telemetry.addData("Battery (V)", "%.2f", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addLine("--- Servos ---");
            telemetry.addData("blocker (left stick Y)", "%.3f", blockerPos);
            telemetry.addData("flowerBar (dpad up/down)", "%.3f", flowerBar.getPosition());
            telemetry.update();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        intake.setPower(0);
        flywheel.setPower(0);
    }

    private double stepServo(double position, double stick, double dtSeconds) {
        if (Math.abs(stick) < STICK_DEADBAND) return position;
        return Range.clip(position + stick * SERVO_SPEED_PER_SEC * dtSeconds, 0.0, 1.0);
    }

    private void motorLine(String label, DcMotor motor) {
        telemetry.addData(label, "%.2f | %d", motor.getPower(), motor.getCurrentPosition());
    }
}
