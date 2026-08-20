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
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSub;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto", group = "Autonomous")
public class PedroAuto extends LinearOpMode {

    private IntakeSub intake;
    private OuttakeSub outtake;
    private IndexerSub indexer;
    private DriveSub drive;

    private Follower follower;
    private Timer actionTimer;
    private int pathState;

    private final Pose startPose = new Pose(22.939, 119.084, Math.toRadians(90));

    // Exact 11 path chains from your setup
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22.939, 119.084), new Pose(60.832, 80.007)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.832, 80.007), new Pose(40.571, 82.296)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(40.571, 82.296), new Pose(15.453, 82.517)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.453, 82.517), new Pose(60.758, 80.156)))
                .setLinearHeadingInterpolation(Math.toRadians(185), Math.toRadians(315))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.758, 80.156), new Pose(40.335, 58.940)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(40.335, 58.940), new Pose(15.751, 58.460)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.751, 58.460), new Pose(60.618, 79.688)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.618, 79.688), new Pose(38.864, 35.096)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(38.864, 35.096), new Pose(14.764, 35.299)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14.764, 35.299), new Pose(60.738, 80.444)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.738, 80.444), new Pose(38.411, 33.240)))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1, true);
                indexer.resetIndexState();
                setPathState(1);
                break;

            case 1:
                // Outtake at 1st location
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    if (indexer.autonomyouttake()) {
                        follower.followPath(path2, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Move straight to path4 without intake
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    indexer.resetIndexState();
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                // Outtake at 2nd location
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    if (indexer.autonomyouttake()) {
                        follower.followPath(path5, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Move straight to path7 without intake
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    indexer.resetIndexState();
                    follower.followPath(path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                // Outtake at 3rd location
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    if (indexer.autonomyouttake()) {
                        follower.followPath(path8, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    follower.followPath(path9, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Move straight to path10 without intake
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    indexer.resetIndexState();
                    follower.followPath(path10, true);
                    setPathState(10);
                }
                break;

            case 10:
                // Final Outtake location
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    if (indexer.autonomyouttake()) {
                        follower.followPath(path11, true);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > 0.2) {
                    setPathState(-1); // Finished
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        actionTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Instantiate Subsystems
        intake = new IntakeSub(hardwareMap);
        outtake = new OuttakeSub(hardwareMap);
        drive = new DriveSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap);

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