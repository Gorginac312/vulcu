package org.firstinspires.ftc.teamcode.autonomy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.SubSystems.DriveSub;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto Test 2", group = "Autonomous")
public class pedroautotest2 extends LinearOpMode {

    private IndexerSub indexer;
    private DriveSub drive;

    private Follower follower;
    private Timer actionTimer;
    private int pathState;
    private boolean isStateInit = true;

    private final Pose startPose = new Pose(22.482, 118.459, Math.toRadians(90));

    private PathChain path1, path2, path4, path6;
    private void followReturnPath() {
        PathChain returnPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        follower.getPose(),
                        new Pose(60, 81)
                ))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        Math.toRadians(315)
                )
                .build();

        follower.followPath(returnPath, true);
    }

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.482, 118.459), new Pose(60, 81)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 81), new Pose(42, 83)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();


        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 81), new Pose(40, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();


        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 81), new Pose(40, 35)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                indexer.resetOuttakeState();
                setPathState(1);
                break;

            case 1:
                if (isStateInit) {
                    isStateInit = false;
                }
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    if (indexer.autonomyouttake()) {
                        follower.followPath(path2, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (isStateInit) {
                    isStateInit = false;
                    indexer.startAutonomyIntake2();
                }
                boolean indexerFinished = indexer.autonomyintake2();

                if (indexerFinished || (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.5)) {

                    indexer.resetOuttakeState();

                    followReturnPath();
                    setPathState(3);
                }
                break;

            case 3:
                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {
                        follower.followPath(path4, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (isStateInit) {
                    isStateInit = false;
                    indexer.startAutonomyIntake2();
                }

                boolean indexerFinished4 = indexer.autonomyintake2();

                if (indexerFinished4 || (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.5)) {

                    indexer.resetOuttakeState();

                    followReturnPath();
                    setPathState(5);
                }
                break;

            case 5:
                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {
                        follower.followPath(path6, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (isStateInit) {
                    isStateInit = false;
                    indexer.startAutonomyIntake2();
                }
                boolean indexerFinished6 = indexer.autonomyintake2();


                if (indexerFinished6 || (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.5)) {

                    indexer.resetOuttakeState();

                    followReturnPath();
                    setPathState(7);
                }
                break;

            case 7:
                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy() &&
                        actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {
                        setPathState(-1);
                    }
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int newstate) {
        pathState = newstate;
        actionTimer.resetTimer();
        isStateInit = true;
    }

    @Override
    public void runOpMode() {
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Instantiate Subsystems
        drive = new DriveSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap , drive);

        buildPaths();

        waitForStart();

        setPathState(0);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}