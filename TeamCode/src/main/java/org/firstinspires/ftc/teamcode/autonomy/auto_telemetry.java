package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Autonomous(name = "Pinpoint Telemetry GOOD", group = "Test")
public class auto_telemetry extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private boolean lastA = false;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Your measured offsets in cm.
        // Make sure the signs are correct for your robot.
        pinpoint.setOffsets(10.5, 18.0, DistanceUnit.CM);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint reset.");
        telemetry.addLine("Keep robot still until READY.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        pinpoint.update();

        telemetry.addLine("INIT");
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Heading continuous deg", "%.2f", getHeadingDeg());
        telemetry.addData("Heading 0-360 deg", "%.2f", normalize360(getHeadingDeg()));
        telemetry.addData("Turn deg/s", "%.2f", pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("X cm", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("Y cm", "%.2f", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();

        if (gamepad1.a && !lastA) {
            pinpoint.resetPosAndIMU();
        }
        lastA = gamepad1.a;

        double headingDeg = getHeadingDeg();

        telemetry.addLine("RUNNING");
        telemetry.addData("Status", pinpoint.getDeviceStatus());

        telemetry.addData("Heading continuous deg", "%.2f", headingDeg);
        telemetry.addData("Heading 0-360 deg", "%.2f", normalize360(headingDeg));
        telemetry.addData("Turn deg/s", "%.2f",
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

        telemetry.addData("X cm", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("Y cm", "%.2f", pinpoint.getPosY(DistanceUnit.CM));

        telemetry.addLine("Press A to reset.");
        telemetry.update();
    }

    private double getHeadingDeg() {
        return pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
    }

    private double normalize360(double angleDeg) {
        angleDeg %= 360.0;
        if (angleDeg < 0.0) angleDeg += 360.0;
        return angleDeg;
    }
}