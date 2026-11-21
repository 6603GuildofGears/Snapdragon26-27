package org.firstinspires.ftc.teamcode.pedro.Auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Mega Auto", group = "Autonomous")
public class Mega_Auto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(88, 130, 270);
    private final Pose startshoottocornerintake = poseFactory.of(120, 130, 0);
    private final Pose startshoottocornerintakeControl1 = poseFactory.of(100, 100, 0);
    private final Pose cornerintaketoshoot2Start = poseFactory.of(120, 130, 0);
    private final Pose cornerintaketoshoot2 = poseFactory.of(135, 130, 0);
    private final Pose cornerintaketoshoot2bridgeStart = poseFactory.of(135, 132, 0);
    private final Pose cornerintaketoshoot2bridge = poseFactory.of(84, 120, 90);
    private final Pose bridge1toshoot2Start = poseFactory.of(84, 120, 90);
    private final Pose bridge1toshoot2 = poseFactory.of(84, 8, 90);
    private final Pose shoot2toflowerbridgeStart = poseFactory.of(84, 8, 90);
    private final Pose shoot2toflowerbridge = poseFactory.of(84, 24, 270);
    private final Pose bridge2toflower = poseFactory.of(96, 8, 270);
    private final Pose bridge2toflowerControl1 = poseFactory.of(96, 24, 0);
    private final Pose flowertoshoot2birdgeStart = poseFactory.of(96, 8, 270);
    private final Pose flowertoshoot2birdge = poseFactory.of(84, 24, 90);
    private final Pose flowertoshoot2birdgeControl1 = poseFactory.of(96, 24, 0);
    private final Pose bridgetoshoot2afterflower = poseFactory.of(84, 8, 90);
    private final Pose thirdshottopark = poseFactory.of(136, 30, 90);
    private final Pose thirdshottoparkControl1 = poseFactory.of(84, 30, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, startshoottocornerintake()),
            follow(follower, cornerintaketoshoot2()),
            follow(follower, cornerintaketoshoot2bridge()),
            follow(follower, bridge1toshoot2()),
            follow(follower, shoot2toflowerbridge()),
            follow(follower, bridge2toflower()),
            follow(follower, flowertoshoot2birdge()),
            follow(follower, bridgetoshoot2afterflower()),
            follow(follower, thirdshottopark())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
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
    }

    public Path startshoottocornerintake() {
        return Paths.curve(start, startshoottocornerintakeControl1, startshoottocornerintake).linear(start, startshoottocornerintake);
    }

    public Path cornerintaketoshoot2() {
        return Paths.line(cornerintaketoshoot2Start, cornerintaketoshoot2).linear(cornerintaketoshoot2Start, cornerintaketoshoot2);
    }

    public Path cornerintaketoshoot2bridge() {
        return Paths.line(cornerintaketoshoot2bridgeStart, cornerintaketoshoot2bridge).linear(cornerintaketoshoot2bridgeStart, cornerintaketoshoot2bridge);
    }

    public Path bridge1toshoot2() {
        return Paths.line(bridge1toshoot2Start, bridge1toshoot2).linear(bridge1toshoot2Start, bridge1toshoot2);
    }

    public Path shoot2toflowerbridge() {
        return Paths.line(shoot2toflowerbridgeStart, shoot2toflowerbridge).linear(shoot2toflowerbridgeStart, shoot2toflowerbridge);
    }

    public Path bridge2toflower() {
        return Paths.curve(shoot2toflowerbridge, bridge2toflowerControl1, bridge2toflower).constant(bridge2toflower);
    }

    public Path flowertoshoot2birdge() {
        return Paths.curve(flowertoshoot2birdgeStart, flowertoshoot2birdgeControl1, flowertoshoot2birdge).linear(flowertoshoot2birdgeStart, flowertoshoot2birdge);
    }

    public Path bridgetoshoot2afterflower() {
        return Paths.line(flowertoshoot2birdge, bridgetoshoot2afterflower).constant(bridgetoshoot2afterflower);
    }

    public Path thirdshottopark() {
        return Paths.curve(bridgetoshoot2afterflower, thirdshottoparkControl1, thirdshottopark).constant(thirdshottopark);
    }
}
