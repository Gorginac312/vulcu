
package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.DriveSub;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;
import org.firstinspires.ftc.teamcode.SubSystems.LimelightSub;

@Autonomous(name = "autonomous_close_all", group = "Test")
public class autonomous_close_all extends LinearOpMode {
    private static final double kP_XY = 0.010;
    private static final double kP_HEADING = 0.010;

    private static final double POSITION_TOLERANCE_CM = 3.0;
    private static final double HEADING_TOLERANCE_DEG = 4.0;

    private static final double MIN_DRIVE_POWER = 0.08;
    private static final double MIN_TURN_POWER = 0.06;

    private static final double MAX_TURN_POWER = 0.18;

    private DcMotor RF, RB, LF, LB;
    private GoBildaPinpointDriver pinpoint;
    private IndexerSub indexer;
    private DriveSub drive;
    private LimelightSub limelight;
    ElapsedTime timeout = new ElapsedTime();
    private boolean limelightAlign(double timeoutSeconds, double noTargetSkipSeconds) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stableTimer = new ElapsedTime();
        ElapsedTime noTargetTimer = new ElapsedTime();

        boolean wasStable = false;
        boolean hasSeenTarget = false;

        timer.reset();
        stableTimer.reset();
        noTargetTimer.reset();

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            limelight.update();

            if (limelight.getresult() == null || !limelight.getresult().isValid()) {
                stopDrive();

                telemetry.addLine("Limelight: no target");
                telemetry.update();

                if (!hasSeenTarget && noTargetTimer.seconds() > noTargetSkipSeconds) {
                    return false;
                }

                wasStable = false;
                stableTimer.reset();
                continue;
            }

            hasSeenTarget = true;
            noTargetTimer.reset();

            double turnCorrection = limelight.getHeadingCorrection(true);
            double forwardCorrection = limelight.getTargetCorrection(true);

            turnCorrection = clip(turnCorrection, -0.18, 0.18);
            forwardCorrection = clip(forwardCorrection, -0.14, 0.14);

            boolean aligned =
                    Math.abs(turnCorrection) < 0.025 &&
                            Math.abs(forwardCorrection) < 0.025;

            if (aligned) {
                if (!wasStable) {
                    stableTimer.reset();
                    wasStable = true;
                }

                if (stableTimer.seconds() > 0.20) {
                    stopDrive();
                    return true;
                }
            } else {
                wasStable = false;
                stableTimer.reset();
            }

            drive.drive(
                    forwardCorrection,
                    0,
                    turnCorrection,
                    1.0
            );

            telemetry.addLine("Limelight aligning...");
            telemetry.addData("tx", "%.2f", limelight.gettx());
            telemetry.addData("turn correction", "%.3f", turnCorrection);
            telemetry.addData("forward correction", "%.3f", forwardCorrection);
            telemetry.update();
        }

        stopDrive();
        return false;
    }@Override
    public void runOpMode() {
        indexer = new IndexerSub(hardwareMap);
        limelight = new LimelightSub(hardwareMap);
        drive = new DriveSub(hardwareMap);


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

        pinpoint.setOffsets(10.5, 18, DistanceUnit.CM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        telemetry.addLine("Resetting Pinpoint. Keep robot still...");
        telemetry.update();

        pinpoint.resetPosAndIMU();

        sleep(1000);

        pinpoint.setPosition(
                new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0)
        );

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            driveToPose(-76, 35, 84, 0.3);
            boolean aligned = limelightAlign(1.5, 0.35);
            telemetry.addData("Limelight aligned", aligned);
            telemetry.update();
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyouttake() && timeout.seconds() < 5) {
                telemetry.addLine("Outtaking...");
                telemetry.update();
            }
            driveToPose(121, 65, 310, 0.3);//first row
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyintake() && timeout.seconds() < 5) {
                telemetry.addLine("Intaking...");
                telemetry.update();
            }
            driveToPose(-76, 35, 84, 0.3);
            aligned = limelightAlign(1.5, 0.35);
            telemetry.addData("Limelight aligned", aligned);
            telemetry.update();
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyouttake() && timeout.seconds() < 5) {
                telemetry.addLine("Outtaking...");
                telemetry.update();
            }
            driveToPose(160, 110, 310, 0.3);//second row
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyintake() && timeout.seconds() < 5) {
                telemetry.addLine("Intaking...");
                telemetry.update();
            }
            driveToPose(-76, 35, 84, 0.3);//outtake
            aligned = limelightAlign(1.5, 0.35);
            telemetry.addData("Limelight aligned", aligned);
            telemetry.update();
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyouttake() && timeout.seconds() < 5) {
                telemetry.addLine("Outtaking...");
                telemetry.update();
            }
            driveToPose(201, 156, 310, 0.3);//third row
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyintake() && timeout.seconds() < 5) {
                telemetry.addLine("Intaking...");
                telemetry.update();
            }
            driveToPose(-76, 35, 84, 0.3);
            aligned = limelightAlign(1.5, 0.35);
            telemetry.addData("Limelight aligned", aligned);
            telemetry.update();
            timeout.reset();
            while (opModeIsActive() && !indexer.autonomyouttake() && timeout.seconds() < 5) {
                telemetry.addLine("Outtaking...");
                telemetry.update();
            }

            stopDrive();
        }
    }
    private void driveToPose(double targetXcm, double targetYcm, double targetHeadingDeg, double maxPower) {

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();

            double currentX = pose.getX(DistanceUnit.CM);
            double currentY = pose.getY(DistanceUnit.CM);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetXcm - currentX;
            double errorY = targetYcm - currentY;
            double errorHeading = angleWrap(targetHeadingDeg - currentHeading);

            double distanceError = Math.hypot(errorX, errorY);

            boolean atPosition = distanceError < POSITION_TOLERANCE_CM;
            boolean atHeading = Math.abs(errorHeading) < HEADING_TOLERANCE_DEG;

            if (atPosition && atHeading) {
                break;
            }

            double headingRad = Math.toRadians(currentHeading);

            double robotStrafeError =
                    errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);

            double robotForwardError =
                    -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

            double strafePower = 0.0;
            double forwardPower = 0.0;
            double turnPower = 0.0;

            if (!atPosition) {
                strafePower = robotStrafeError * kP_XY;
                forwardPower = robotForwardError * kP_XY;

                strafePower = clip(strafePower, -maxPower, maxPower);
                forwardPower = clip(forwardPower, -maxPower, maxPower);

                // Only use minimum power when we are not super close
                if (distanceError > 8.0) {
                    if (strafePower != 0 && Math.abs(strafePower) < MIN_DRIVE_POWER) {
                        strafePower = Math.signum(strafePower) * MIN_DRIVE_POWER;
                    }

                    if (forwardPower != 0 && Math.abs(forwardPower) < MIN_DRIVE_POWER) {
                        forwardPower = Math.signum(forwardPower) * MIN_DRIVE_POWER;
                    }
                }
            }

            if (!atHeading) {
                turnPower = errorHeading * kP_HEADING;

                turnPower = clip(turnPower, -MAX_TURN_POWER, MAX_TURN_POWER);

                if (turnPower != 0 && Math.abs(turnPower) < MIN_TURN_POWER) {
                    turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
                }
            }

            drive.drive(forwardPower,
                    strafePower,
                    turnPower,
                    1.0
            );

            telemetry.addData("Target X cm", targetXcm);
            telemetry.addData("Target Y cm", targetYcm);
            telemetry.addData("Current X cm", "%.2f", currentX);
            telemetry.addData("Current Y cm", "%.2f", currentY);
            telemetry.addData("Heading deg", "%.2f", currentHeading);
            telemetry.addData("Error X cm", "%.2f", errorX);
            telemetry.addData("Error Y cm", "%.2f", errorY);
            telemetry.addData("Distance Error cm", "%.2f", distanceError);
            telemetry.addData("Error Heading", "%.2f", errorHeading);
            telemetry.addData("Forward Power", "%.2f", forwardPower);
            telemetry.addData("Strafe Power", "%.2f", strafePower);
            telemetry.addData("Turn Power", "%.2f", turnPower);
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