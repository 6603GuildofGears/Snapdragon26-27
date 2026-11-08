package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Blue version of Paths: every red point mirrored over both axes of the 144 x 144 field,
// i.e. (x, y, heading) -> (144 - x, 144 - y, heading + 180).
public class BluePaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(88, 136, 270);
    private final Pose starttoIntakeBridge = poseFactory.of(128, 135, 0);
    private final Pose starttoIntakeBridgeControl1 = poseFactory.of(88, 124, 180);
    private final Pose intakePathStart = poseFactory.of(128, 135, 0);
    private final Pose intakePath = poseFactory.of(135, 135, 0);
    private final Pose intaketoShootStart = poseFactory.of(135, 135, 0);
    private final Pose intaketoShoot = poseFactory.of(88, 136, 270);
    private final Pose intaketoShootControl1 = poseFactory.of(88, 124, 180);
    private final Pose shoottoParkStart = poseFactory.of(88, 136, 270);
    private final Pose shoottoPark = poseFactory.of(131.2788, 24.4746, 270);
    private final Pose shoottoParkControl1 = poseFactory.of(129.609, 88.8454, 180);

    public Pose startPose() {
        return start;
    }

    public Path starttoIntakeBridge() {
        return com.pedropathing.api.Paths.curve(start, starttoIntakeBridgeControl1, starttoIntakeBridge).linear(start, starttoIntakeBridge);
    }

    public Path intakePath() {
        return com.pedropathing.api.Paths.line(intakePathStart, intakePath).linear(intakePathStart, intakePath);
    }

    public Path intaketoShoot() {
        return com.pedropathing.api.Paths.curve(intaketoShootStart, intaketoShootControl1, intaketoShoot).linear(intaketoShootStart, intaketoShoot);
    }

    public Path shoottoPark() {
        return com.pedropathing.api.Paths.curve(shoottoParkStart, shoottoParkControl1, shoottoPark).linear(shoottoParkStart, shoottoPark);
    }
}
