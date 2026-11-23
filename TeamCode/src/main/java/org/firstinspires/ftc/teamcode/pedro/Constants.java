package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontleftmotor");
        c.frontRightName.set("frontrightmotor");
        c.backLeftName.set("backleftmotor");
        c.backRightName.set("backrightmotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-11.359649417907235);
        c.yPodOffset.set(5.8958837554210755);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22908739527237001);
                Controller secondaryTranslationalForward = Controller.proportional(0.08464167122520092);
                Controller primaryTranslationalLateral = Controller.proportional(0.239698326672393);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08856212684820058);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01676480095096104));
                c.brake.set(Controller.proportionalFeedforward(0.014250080808316882 * 1.5));

                c.headingFeedback.set(Controller.proportional(2.9916847987934494));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0493050285221533 * 1.5, 0.00394912059464234 * 1.5));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0489971469831598 * 1.5, 0.069065925658806 * 1.5));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020568915851435342 * 1.5, 0.0014691025366729165 * 1.5));
                c.maxBrakingPower.set(0.6);

                c.maxPathSpeed.set(0.6); // Cap target path speed to 60% of max speed (was 0.3)
                c.maxVelocityConstraint.set(30.0); // Cap velocity to 30 inches/second (was 15)

                c.maxAchievableForwardVelocity.set(61.961372145639096);
                c.maxAchievableStrafeVelocity.set(54.10417246228246);
                c.naturalForwardDeceleration.set(23.153745180804354);
                c.naturalStrafeDeceleration.set(28.586856343242257);
            }
    );

    /**
     * Custom Localizer Wrapper that transforms the coordinate system in software:
     * - Moving FORWARD manually -> Y INCREASES
     * - Moving BACKWARD manually -> Y DECREASES
     * - Moving RIGHT manually -> X INCREASES
     * - Moving LEFT manually -> X DECREASES
     */
    public static class SwappedPinpointLocalizer implements Localizer {
        private final PinpointLocalizer base;

        public SwappedPinpointLocalizer(HardwareMap h, PinpointConfig config) {
            this.base = new PinpointLocalizer(h, config);
        }

        @Override
        public void setPose(Pose pose) {
            // Map desired (X=Right, Y=Forward) to base Pinpoint (BaseX=Forward, BaseY=Right)
            base.setPose(new Pose(pose.y(), pose.x(), pose.heading()));
        }

        @Override
        public void update() {
            base.update();
        }

        @Override
        public void reset() {
            base.reset();
        }

        @Override
        public MotionState state() {
            MotionState baseState = base.state();
            if (baseState == null) return null;
            Pose basePose = baseState.pose();
            Velocity baseVel = baseState.velocity();

            // Transform axes: Pinpoint Forward (basePose.x) -> Y, Pinpoint Strafe (basePose.y) -> X
            Pose swappedPose = new Pose(basePose.y(), basePose.x(), basePose.heading());
            Velocity swappedVel = new Velocity(baseVel.vy, baseVel.vx, baseVel.omega);

            return MotionState.ofVelocity(swappedPose, swappedVel);
        }
    }

    /**
     * Uses the standard Pinpoint localizer. With a starting heading of 90°,
     * Pedro's normal field frame already gives Y = forward and X = right,
     * so no axis swap is needed. (SwappedPinpointLocalizer swapped X/Y while
     * still passing the 90° heading to the Pinpoint, which made forward motion
     * read as sideways and caused the robot to drive diagonally.)
     */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createStandard(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
