package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Autonomous(name = "autonomy_main   ", group = "Test")
public class autonomy_main extends OpMode {

    GoBildaPinpointDriver pinpoint;
    boolean lastA = false;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(10.5, 18, DistanceUnit.CM);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
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
        telemetry.addData("Turn deg/s", "%.2f",
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("X cm", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("Y cm", "%.2f", pinpoint.getPosY(DistanceUnit.CM));
        telemetry.update();
    }
}