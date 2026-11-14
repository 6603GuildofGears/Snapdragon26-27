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
    c.manualBrakeMode.set(true);
    
});

public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    c.xPodOffset.set(3.6254155917430486);
    c.yPodOffset.set(4.456540806086983);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});

public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3822565513434427);
                Controller secondaryTranslationalForward = Controller.proportional(0.1412335816382347);
                Controller primaryTranslationalLateral = Controller.proportional(0.6526534608947158);
                Controller secondaryTranslationalLateral = Controller.proportional(0.24113801457893963);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011319947350707397));
                c.brake.set(Controller.proportionalFeedforward(0.009621955248101288));

                c.headingFeedback.set(Controller.proportional(7.244784967515984));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06927189244886375, 0.005515791466923878));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13795078940646063, 0.08963283157708914));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010140193155431573, 0.0014993204353605442));

                c.maxAchievableForwardVelocity.set(84.00993064087032);
                c.maxAchievableStrafeVelocity.set(63.717043755880965);
                c.naturalForwardDeceleration.set(37.493441800621575);
                c.naturalStrafeDeceleration.set(81.5519920391556);
            }
    );

}