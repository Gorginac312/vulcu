    package org.firstinspires.ftc.teamcode.pedroPathing;

    import com.pedropathing.control.FilteredPIDFCoefficients;
    import com.pedropathing.control.PIDFCoefficients;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.follower.FollowerConstants;
    import com.pedropathing.ftc.FollowerBuilder;
    import com.pedropathing.ftc.drivetrains.MecanumConstants;
    import com.pedropathing.ftc.localization.constants.PinpointConstants;
    import com.pedropathing.paths.PathConstraints;

    import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;

    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

    public class Constants {

        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(13.3)
                .forwardZeroPowerAcceleration(-30.97324)
                .lateralZeroPowerAcceleration(-62.6329)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.09,0.0,0.002,0.02))
                .headingPIDFCoefficients(new PIDFCoefficients(1,0.0,0.001,0.02));
        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .rightFrontMotorName("RF")
                .rightRearMotorName("RB")
                .leftRearMotorName("LB")
                .leftFrontMotorName("LF")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

                .xVelocity(72.0454)
                .yVelocity(52.0249);

        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-4.5)
                .strafePodX(-6)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .yawScalar(1.009);

        public static PathConstraints pathConstraints =
                new PathConstraints(0.99, 100, 0.9, 1.1);

        public static PathConstraints intakePathConstraints =
                new PathConstraints(0.35, 50, 0.9, 1.1);

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .mecanumDrivetrain(driveConstants)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .build();
        }
    }