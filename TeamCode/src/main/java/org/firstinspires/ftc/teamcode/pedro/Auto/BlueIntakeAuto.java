package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Shooter;

@Autonomous(name = "Blue Intake Auto", group = "Autonomous")
public class BlueIntakeAuto extends LinearOpMode {
    static final double INTAKE_POWER = 1.0;

    private Follower follower;
    private Shooter shooter;
    private BluePaths paths;

    // Shoot, intake (bridge, intake path, back up and return to the same spot), back to shoot, shoot,
    // flower (bar down + intake, drive in, strafe, bar up on the way back), shoot, park
    private Command autoRoutine() {
        return sequential(
                shooter.shoot(),
                sequential(
                        instant(() -> shooter.setIntakePower(INTAKE_POWER)),
                        follow(follower, paths.starttoIntakeBridge()),
                        follow(follower, paths.intakePath()),
                        follow(follower, paths.intakeBackup()),
                        follow(follower, paths.intakeReturn()),
                        instant(() -> shooter.setIntakePower(0))
                ),
                follow(follower, paths.intaketoShoot()),
                shooter.shoot(),
                sequential(
                        instant(() -> {
                            shooter.setFlowerBar(true);
                            shooter.setIntakePower(INTAKE_POWER);
                        }),
                        follow(follower, paths.shoottoFlower1()),
                        follow(follower, paths.flower1Strafe()),
                        instant(() -> shooter.setFlowerBar(false)),
                        follow(follower, paths.flower1toShoot()),
                        instant(() -> shooter.setIntakePower(0))
                ),
                shooter.shoot(),
                follow(follower, paths.shoottoPark())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        shooter = new Shooter(hardwareMap);
        paths = new BluePaths();
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose());
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
