package org.firstinspires.ftc.teamcode.autonomy;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "auto_pedro_close", group = "Pedro")
public class auto_pedro_close extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;

    private int pathState = -1;

    // Pedro uses field inches: 0..144.
    // Headings are in radians, so use Math.toRadians().
    private final Pose startPose       = new Pose(22, 118, Math.toRadians(180));
    private final Pose shootPose       = new Pose(48, 93,  Math.toRadians(130));
    private final Pose pickupStartPose = new Pose(35, 82,  Math.toRadians(190));
    private final Pose pickup1Pose     = new Pose(38, 85,  Math.toRadians(180));
    private final Pose parkPose        = new Pose(60, 105, Math.toRadians(90));

    private PathChain scorePreload;
    private PathChain goToPickupStart;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        shootPose
                ))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        // Shooting position -> pickup start
        goToPickupStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        pickupStartPose
                ))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        pickupStartPose.getHeading()
                )
                .build();

        // Pickup start -> first pickup point
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickupStartPose,
                        pickup1Pose
                ))
                .setConstantHeadingInterpolation(
                        pickupStartPose.getHeading()
                )
                .build();

        // Pickup point -> shooting position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pose,
                        new Pose(42, 92),
                        shootPose
                ))
                .setLinearHeadingInterpolation(
                        pickup1Pose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        // Shooting position -> park
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootPose,
                        parkPose
                ))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        parkPose.getHeading()
                )
                .build();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Go shoot preload.
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (pathFinished(4.0)) {
                    // TODO: Shoot preload here.
                    // For first testing, this just waits 1 second.
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(goToPickupStart, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathFinished(3.0)) {
                    follower.followPath(grabPickup1, 0.2 , true);
                    setPathState(4);
                }
                break;

            case 4:
                if (pathFinished(2.0)) {
                    // TODO: Turn intake on here if needed.
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathFinished(4.0)) {
                    // TODO: Shoot picked-up balls here.
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (pathFinished(3.0)) {
                    setPathState(-1);
                }
                break;
        }
    }

    private boolean pathFinished(double timeoutSeconds) {
        return !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > timeoutSeconds;
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addLine("Pedro auto ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading deg", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("busy", follower.isBusy());
            telemetry.update();
        }
    }
}