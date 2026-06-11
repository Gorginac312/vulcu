package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Autonomous(name = "autonomous_telemetry", group = "Test")
public class autonomous_telemetry extends OpMode {

    GoBildaPinpointDriver pinpoint;
    boolean lastA = false;
    private double getHeadingDeg() {
        return pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
    }
    private double normalize360(double angleDeg) {
        angleDeg %= 360.0;
        if (angleDeg < 0.0) angleDeg += 360.0;
        return angleDeg;
    }
    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(10.5, 18, DistanceUnit.CM);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Resetting IMU. Keep robot still.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        pinpoint.update();

        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Heading deg", "%.2f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Turn deg/s", "%.2f",
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();

        if (gamepad1.a && !lastA) {
            pinpoint.resetPosAndIMU();
        }
        lastA = gamepad1.a;

        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Heading deg", "%.2f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading 0-360 deg", "%.2f", normalize360(getHeadingDeg()));
        telemetry.addData("X cm", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("Y cm", "%.2f", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.update();

        return 0;
    }
}