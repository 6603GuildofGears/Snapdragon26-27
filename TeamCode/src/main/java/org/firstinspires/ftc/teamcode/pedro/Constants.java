package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

public class Constants {
  public static Follower create(HardwareMap h) {
    return new Follower(
        new PinpointLocalizer(h, localizerConfig),
        new Mecanum(h, drivetrainConfig),
        new Foresight(foresightConfig)
    );
}

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    c.xPodOffset.set(5.7219714067113685);
    c.yPodOffset.set(0.6957878653458723);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});

public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.36162982463560805);
                Controller secondaryTranslationalForward = Controller.proportional(0.13361255727597823);
                Controller primaryTranslationalLateral = Controller.proportional(0.654552754147063);
                Controller secondaryTranslationalLateral = Controller.proportional(0.24183975268563151);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011312952251648242));
                c.brake.set(Controller.proportionalFeedforward(0.009616009413901005));

                c.headingFeedback.set(Controller.proportional(7.063187974382037));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07444290159296839, 0.00451972120744038));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10426413770447236, 0.10754591443648441));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019600548150274467, 0.0012789715972687983));

                c.maxAchievableForwardVelocity.set(70.07547723838198);
                c.maxAchievableStrafeVelocity.set(58.8198430651111);
                c.naturalForwardDeceleration.set(39.611071297792385);
                c.naturalStrafeDeceleration.set(75.11516702327286);
            }
    );
}