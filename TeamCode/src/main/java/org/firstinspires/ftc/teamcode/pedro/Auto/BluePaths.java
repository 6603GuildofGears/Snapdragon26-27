package org.firstinspires.ftc.teamcode.pedro.TeleOp.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

// Blue version of Paths: every red point mirrored over both axes of the 144 x 144 field,
// i.e. (x, y, heading) -> (144 - x, 144 - y, heading + 180).
public class BluePaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(88, 136, 270);
    private final Pose intakeBridge = poseFactory.of(128, 132, 30);
    private final Pose starttoIntakeBridgeControl1 = poseFactory.of(100, 100, 0);
    private final Pose intakeStart = poseFactory.of(128, 130, 30);
    private final Pose intakeEnd = poseFactory.of(130, 130, 90);
    private final Pose intakeBack = poseFactory.of(130, 125, 90);
    private final Pose intakeExit = poseFactory.of(130, 131, 0);
    private final Pose shoot = poseFactory.of(88, 136, 270);
    private final Pose intaketoShootControl1 = poseFactory.of(88, 124, 180);
    private final Pose shootExit = poseFactory.of(88, 136, 270);
    private final Pose park = poseFactory.of(131.2788, 24.4746, 270);
    private final Pose shoottoParkControl1 = poseFactory.of(129.609, 88.8454, 180);

    private final Pose flower1 = poseFactory.of(130, 96, 0);
    // A few inches sideways from flower1, same heading. Flip the sign of the y offset if it goes the wrong way.
    private final Pose flower1intakeend = poseFactory.of(130, 100, 0);

    public Pose startPose() {
        return start;
    }

    public Path starttoIntakeBridge() {
        return com.pedropathing.api.Paths.curve(start, starttoIntakeBridgeControl1, intakeBridge).linear(start, intakeBridge);
    }

    public Path intakePath() {
        return com.pedropathing.api.Paths.line(intakeStart, intakeEnd).linear(intakeStart, intakeEnd);
    }

    // Back up and return are two straight legs: one curve out and back along a line has a
    // zero-velocity point at the turnaround, which makes Pedro normalize a zero vector.
    public Path intakeBackup() {
        return com.pedropathing.api.Paths.line(intakeEnd, intakeBack).linear(intakeEnd, intakeBack);
    }

    public Path intakeReturn() {
        return com.pedropathing.api.Paths.line(intakeBack, intakeExit).linear(intakeBack, intakeExit);
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

    public Path flower1toShoot() {
        return com.pedropathing.api.Paths.line(flower1intakeend, shoot).linear(flower1intakeend, shoot);
    }

    public Path shoottoPark() {
        return com.pedropathing.api.Paths.curve(shootExit, shoottoParkControl1, park).linear(shootExit, park);
    }
}
