package org.firstinspires.ftc.teamcode.pedro.Auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Shooter;

@Autonomous(name = "Down_One", group = "Autonomous")
public class Down_One extends LinearOpMode {
    static final double SHOOT_DELAY_SEC = 10.0;

    private Follower follower;
    private Shooter shooter;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(79, 8, 90);
    private final Pose path1 = poseFactory.of(132, 26, 180);
    private final Pose path1Control1 = poseFactory.of(85.5216, 24.5635, 0);

    // Wait, shoot, then park
    public Command autoRoutine() {
        return sequential(
            waitMs(SHOOT_DELAY_SEC * 1000),
            shooter.shoot(),
            follow(follower, path1())
        );
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

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        shooter.stop();
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
