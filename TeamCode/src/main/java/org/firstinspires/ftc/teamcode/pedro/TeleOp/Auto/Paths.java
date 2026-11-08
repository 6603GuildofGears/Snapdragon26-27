package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Generated paths. Methods call com.pedropathing.api.Paths by full name because this class is also called Paths.
public class Paths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose starttoIntakeBridge = poseFactory.of(16, 9, 180);
    private final Pose starttoIntakeBridgeControl1 = poseFactory.of(56, 20, 0);
    private final Pose intakePathStart = poseFactory.of(16, 9, 180);
    private final Pose intakePath = poseFactory.of(9, 9, 180);
    private final Pose intaketoShootStart = poseFactory.of(9, 9, 180);
    private final Pose intaketoShoot = poseFactory.of(56, 8, 90);
    private final Pose intaketoShootControl1 = poseFactory.of(56, 20, 0);
    private final Pose shoottoParkStart = poseFactory.of(56, 8, 90);
    private final Pose shoottoPark = poseFactory.of(12.7212, 119.5254, 90);
    private final Pose shoottoParkControl1 = poseFactory.of(14.391, 55.1546, 0);

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
