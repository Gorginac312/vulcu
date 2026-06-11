
package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;

@Autonomous(name = "autonomous_main", group = "Test")
public class autonomous_close_open extends LinearOpMode {
    private double getHeadingDeg() {
        return pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
    }
    private double normalize360(double angleDeg) {
        angleDeg %= 360.0;
        if (angleDeg < 0.0) angleDeg += 360.0;
        return angleDeg;
    }


    private DcMotor RF, RB, LF, LB;
    private GoBildaPinpointDriver pinpoint;
    private IndexerSub indexer;

    @Override
    public void runOpMode() {
        indexer = new IndexerSub(hardwareMap);

        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(10.5, 18 , DistanceUnit.CM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.addLine("Resetting Pinpoint. Keep robot still...");
        telemetry.update();

        pinpoint.resetPosAndIMU();

        sleep(1000);

        telemetry.addLine("Ready!");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) {
            telemetry.addLine("INIT");
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Heading continuous deg", "%.2f", pinpoint.getHeading());
            telemetry.addData("Heading 0-360 deg", "%.2f", normalize360(getHeadingDeg()));
            telemetry.addData("Turn deg/s", "%.2f", pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("X cm", "%.2f", pinpoint.getPosX(DistanceUnit.CM));
            telemetry.addData("Y cm", "%.2f", pinpoint.getPosY(DistanceUnit.CM));
            telemetry.update();

            driveToPose(-82, 31, 85, 1);
            sleep(3000);
            indexer.autonomyouttake();
            sleep(10000);
            driveToPose(-121, 65, 310, 1);
            sleep(3000);
            indexer.autonomyintake();
            sleep(3000);
            driveToPose(-82, 231, 85, 1);
            sleep(10000);
            indexer.autonomyouttake();
            sleep(3000);
            driveToPose(-160, 110, 310, 1);
            sleep(3000);
            indexer.autonomyintake();
            driveToPose(-82 , 31 , 85 , 1);
            indexer.autonomyouttake();

            stopDrive();
        }
    }

    private void driveToPose(double targetXcm, double targetYcm, double targetHeadingDeg, double maxPower) {

        double kP_XY = 0.014;
        double kP_Heading = 0.015;

        double positionTolerance = 2.5;
        double headingTolerance = 3.0;

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();

            double currentX = pose.getX(DistanceUnit.CM);
            double currentY = pose.getY(DistanceUnit.CM);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetXcm - currentX;
            double errorY = targetYcm - currentY;
            double errorHeading = angleWrap(targetHeadingDeg - currentHeading);

            boolean atX = Math.abs(errorX) < positionTolerance;
            boolean atY = Math.abs(errorY) < positionTolerance;
            boolean atHeading = Math.abs(errorHeading) < headingTolerance;

            if (atX && atY && atHeading) {
                break;
            }

            double headingRad = Math.toRadians(currentHeading);

            double robotX = errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);
            double robotY = -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

            double strafePower = robotX * kP_XY;
            double forwardPower = robotY * kP_XY;
            double turnPower = errorHeading * kP_Heading;

            strafePower = clip(strafePower, -maxPower, maxPower);
            forwardPower = clip(forwardPower, -maxPower, maxPower);
            turnPower = clip(turnPower, -maxPower, maxPower);

            mecanumDrive(forwardPower, strafePower, turnPower);

            telemetry.addData("Target X cm", targetXcm);
            telemetry.addData("Target Y cm", targetYcm);
            telemetry.addData("Current X cm", "%.2f", currentX);
            telemetry.addData("Current Y cm", "%.2f", currentY);
            telemetry.addData("Heading deg", "%.2f", currentHeading);
            telemetry.addData("Error X cm", "%.2f", errorX);
            telemetry.addData("Error Y cm", "%.2f", errorY);
            telemetry.addData("Error Heading", "%.2f", errorHeading);
            telemetry.update();
        }

        stopDrive();
    }
    private void mecanumDrive(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))
        ));

        LF.setPower(lf / max);
        RF.setPower(rf / max);
        LB.setPower(lb / max);
        RB.setPower(rb / max);
    }

    private void stopDrive() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}