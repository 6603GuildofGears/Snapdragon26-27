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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Shooter;

@Autonomous(name = " Intake Auto", group = "Autonomous")
public class IntakeAuto extends LinearOpMode {
    static final double INTAKE_POWER = 1;

    private Follower follower;
    private Shooter shooter;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(88, 136, 270);
    private final Pose intakeBridge = poseFactory.of(120, 133, 0);
    private final Pose starttoIntakeBridgeControl1 = poseFactory.of(100, 100, 0);
    private final Pose intakeStart = poseFactory.of(120, 133, 0);
    private final Pose intakeEnd = poseFactory.of(135, 133, 0);
    private final Pose intakeExit = poseFactory.of(135, 133, 90);
    private final Pose shoot = poseFactory.of(88, 136, 270);
    private final Pose intaketoShootControl1 = poseFactory.of(100, 100, 0);
    private final Pose park = poseFactory.of(130.6749, 47.0326, 180);

    private final Pose flower1 = poseFactory.of(130, 96, 0);
    private final Pose flower1intakeend = poseFactory.of(130, 90, 0);

    // Shoot, intake (bridge, intake path), back to shoot, shoot,
    // flower (bar down + intake, drive in, strafe, bar up, drive to park)
    private Command autoRoutine() {
        return sequential(
                shooter.shoot(),
                sequential(
                        instant(() -> shooter.setIntakePower(INTAKE_POWER)),
                        follow(follower, starttoIntakeBridge()),
                        follow(follower, intakePath()),
                        instant(() -> shooter.setIntakePower(0))
                ),
                follow(follower, intaketoShoot()),
                shooter.shoot(),
                sequential(
                        instant(() -> {
                            shooter.setFlowerBar(true);
                            shooter.setIntakePower(INTAKE_POWER);
                        }),
                        follow(follower, shoottoFlower1()),
                        follow(follower, flower1Strafe()),
                        instant(() -> shooter.setFlowerBar(false)),
                        follow(follower, flower1toPark()),
                        instant(() -> shooter.setIntakePower(0))
                )
        );
    }

    private Path starttoIntakeBridge() {
        return Paths.curve(start, starttoIntakeBridgeControl1, intakeBridge).linear(start, intakeBridge);
    }

    private Path intakePath() {
        return Paths.line(intakeStart, intakeEnd).linear(intakeStart, intakeEnd);
    }

    private Path intaketoShoot() {
        return Paths.curve(intakeExit, intaketoShootControl1, shoot).linear(intakeExit, shoot);
    }

    private Path shoottoFlower1() {
        return Paths.line(shoot, flower1).linear(shoot, flower1);
    }

    private Path flower1Strafe() {
        return Paths.line(flower1, flower1intakeend).linear(flower1, flower1intakeend);
    }

    private Path flower1toPark() {
        return Paths.line(flower1intakeend, park).linear(flower1intakeend, park);
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
