package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class BluePaths {

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

    public Pose startPose() {
        return start;
    }

    public Path starttoIntakeBridge() {
        return com.pedropathing.api.Paths.curve(start, starttoIntakeBridgeControl1, intakeBridge).linear(start, intakeBridge);
    }

    public Path intakePath() {
        return com.pedropathing.api.Paths.line(intakeStart, intakeEnd).linear(intakeStart, intakeEnd);
    }

    public Path intaketoShoot() {
        return com.pedropathing.api.Paths.curve(intakeExit, intaketoShootControl1, shoot).linear(intakeExit, shoot);
    }

    public Path shoottoFlower1() {
        return com.pedropathing.api.Paths.line(shoot, flower1).linear(shoot, flower1);
    }

    public Path flower1Strafe() {
        return com.pedropathing.api.Paths.line(flower1, flower1intakeend).linear(flower1, flower1intakeend);
    }

    public Path flower1toPark() {
        return com.pedropathing.api.Paths.line(flower1intakeend, park).linear(flower1intakeend, park);
    }
}
