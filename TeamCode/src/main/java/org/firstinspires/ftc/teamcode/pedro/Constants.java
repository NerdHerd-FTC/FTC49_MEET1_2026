package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("left_front");
                c.backLeftName.set("left_back");
                c.frontRightName.set("right_front");
                c.backRightName.set("right_back");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.501654256985883);
        c.yPodOffset.set(-4.699191596564345);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    /*public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.935520382378045);
        c.yPodOffset.set(-0.567464790944978);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });*/

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3777865724511563);
                Controller secondaryTranslationalForward = Controller.proportional(0.13958204387757067);
                Controller primaryTranslationalLateral = Controller.proportional(0.5889783728160674);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21761177096356818);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
                c.coast.set(Controller.proportionalFeedforward(0.011418436830370486));
                c.brake.set(Controller.proportionalFeedforward(0.009705671305814914));
                c.headingFeedback.set(Controller.proportional(5.727571033322768));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06875167749113899, 0.005151110183201477));
                c.linearBrakeCoefficients.set(Matrix.diag(0.10955593585448306, 0.0729149295231363));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015448484938494853, 0.001767401625049925));
                c.maxAchievableForwardVelocity.set(82.24161407889699);
                c.maxAchievableStrafeVelocity.set(66.54080508600029);
                c.naturalForwardDeceleration.set(31.642643711858);
                c.naturalStrafeDeceleration.set(67.02830361672655);
            }
    );
    /*public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2938106841025499);
                Controller secondaryTranslationalForward = Controller.proportional(0.10855519700982334);
                Controller primaryTranslationalLateral = Controller.proportional(0.8211715344850358);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3034009398847057);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014644283835584682));
                c.brake.set(Controller.proportionalFeedforward(0.012447641260246979));

                c.headingFeedback.set(Controller.proportional(5.613660843174921));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05119897711382815, 0.008458899408611869));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06549738424054644, 0.05686744879064825));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0024389630983745365, 0.002462454233904857));

                c.maxAchievableForwardVelocity.set(61.209337737973);
                c.maxAchievableStrafeVelocity.set(44.40021459545068);
                c.naturalForwardDeceleration.set(61.80053139878785);
                c.naturalStrafeDeceleration.set(77.73762148312699);

                c.timeoutConstraint.set(50.0);
            }
    );
    */

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}