package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Shooter;

@Disabled
@Autonomous(name = "Red Intake Auto", group = "Autonomous")
public class RedIntakeAuto extends LinearOpMode {
    static final double INTAKE_POWER = 1.0;

    private Follower follower;
    private Shooter shooter;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose starttoIntakeBridge = poseFactory.of(16, 9, 180);
    private final Pose starttoIntakeBridgeControl1 = poseFactory.of(56, 20, 0);
    private final Pose intakePathStart = poseFactory.of(20, 9, 180);
    private final Pose intakePath = poseFactory.of(10, 9, 180);
    private final Pose intaketoShootStart = poseFactory.of(10, 9, 180);
    private final Pose intaketoShoot = poseFactory.of(56, 8, 90);
    private final Pose intaketoShootControl1 = poseFactory.of(56, 20, 0);
    private final Pose shoottoParkStart = poseFactory.of(56, 8, 90);
    private final Pose shoottoPark = poseFactory.of(12.7212, 119.5254, 90);
    private final Pose shoottoParkControl1 = poseFactory.of(14.391, 55.1546, 0);

    // Shoot, bridge, intake, back to shoot, shoot, park
    private Command autoRoutine() {
        return sequential(
                shooter.shoot(),
                follow(follower, starttoIntakeBridge()),
                sequential(
                        instant(() -> shooter.setIntakePower(INTAKE_POWER)),
                        follow(follower, intakePath()),
                        instant(() -> shooter.setIntakePower(0))
                ),
                follow(follower, intaketoShoot()),
                shooter.shoot(),
                follow(follower, shoottoPark())
        );
    }

    private Path starttoIntakeBridge() {
        return Paths.curve(start, starttoIntakeBridgeControl1, starttoIntakeBridge).linear(start, starttoIntakeBridge);
    }

    private Path intakePath() {
        return Paths.line(intakePathStart, intakePath).linear(intakePathStart, intakePath);
    }

    private Path intaketoShoot() {
        return Paths.curve(intaketoShootStart, intaketoShootControl1, intaketoShoot).linear(intaketoShootStart, intaketoShoot);
    }

    private Path shoottoPark() {
        return Paths.curve(shoottoParkStart, shoottoParkControl1, shoottoPark).linear(shoottoParkStart, shoottoPark);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        shooter = new Shooter(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            shooter.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }

        shooter.stop();
    }
}
