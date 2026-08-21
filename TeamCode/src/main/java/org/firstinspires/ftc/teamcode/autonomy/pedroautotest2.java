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

    private final Pose startPose =
            new Pose(22.498, 117.762, Math.toRadians(90));

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;
    private PathChain path10;

    public void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.498, 117.762),
                        new Pose(58.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(315)
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(58.000, 83.000),
                        new Pose(40.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(315),
                        Math.toRadians(180)
                ).

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(40.000, 83.000),
                        new Pose(12.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .setPathConstraints(Constants.intakePathConstraints)
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.000, 83.000),
                        new Pose(58.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(315)
                )
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(58.000, 83.000),
                        new Pose(40.000, 59.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(315),
                        Math.toRadians(180)
                )
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(40.000, 59.000),
                        new Pose(12.000, 59.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .setPathConstraints(Constants.intakePathConstraints)
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.000, 59.000),
                        new Pose(58.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(315)
                )
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(58.000, 83.000),
                        new Pose(40.000, 35.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(315),
                        Math.toRadians(180)
                )
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(40.000, 35.000),
                        new Pose(12.000, 35.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .setPathConstraints(Constants.intakePathConstraints)
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.000, 35.000),
                        new Pose(58.000, 83.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(315)
                )
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            // =========================================================
            // 0: START -> FIRST SHOOTING POSITION
            // =========================================================
            case 0:

                follower.followPath(path1, true);
                indexer.resetOuttakeState();

                setPathState(1);
                break;


            // =========================================================
            // 1: FIRST OUTTAKE
            // =========================================================
            case 1:

                if (!follower.isBusy()
                        && actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {

                        follower.followPath(path2, true);
                        setPathState(2);
                    }
                }
                break;


            // =========================================================
            // 2: GO TO FIRST INTAKE
            // =========================================================
            case 2:

                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy()) {

                    indexer.startAutonomyIntake2();

                    follower.followPath(path3, true);

                    setPathState(3);
                }
                break;


            // =========================================================
            // 3: FIRST INTAKE CREEP
            // Intake runs while Pedro follows path3
            // =========================================================
            case 3:

                indexer.autonomyintake2();

                if (!follower.isBusy()) {

                    indexer.resetOuttakeState();

                    follower.followPath(path4, true);

                    setPathState(4);
                }
                break;


            // =========================================================
            // 4: RETURN TO SHOOTING POSITION
            // =========================================================
            case 4:

                if (!follower.isBusy()
                        && actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {

                        follower.followPath(path5, true);
                        setPathState(5);
                    }
                }
                break;


            // =========================================================
            // 5: GO TO SECOND INTAKE
            // =========================================================
            case 5:

                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy()) {

                    indexer.startAutonomyIntake2();

                    follower.followPath(path6, true);

                    setPathState(6);
                }
                break;


            // =========================================================
            // 6: SECOND INTAKE CREEP
            // =========================================================
            case 6:

                indexer.autonomyintake2();

                if (!follower.isBusy()) {

                    indexer.resetOuttakeState();

                    follower.followPath(path7, true);

                    setPathState(7);
                }
                break;


            // =========================================================
            // 7: RETURN TO SHOOTING POSITION
            // =========================================================
            case 7:

                if (!follower.isBusy()
                        && actionTimer.getElapsedTimeSeconds() > 0.2) {

                    if (indexer.autonomyouttake()) {

                        follower.followPath(path8, true);
                        setPathState(8);
                    }
                }
                break;


            // =========================================================
            // 8: GO TO THIRD INTAKE
            // =========================================================
            case 8:

                if (isStateInit) {
                    isStateInit = false;
                }

                if (!follower.isBusy()) {

                    indexer.startAutonomyIntake2();

                    follower.followPath(path9, true);

                    setPathState(9);
                }
                break;


            // =========================================================
            // 9: THIRD INTAKE CREEP
            // =========================================================
            case 9:

                indexer.autonomyintake2();

                if (!follower.isBusy()) {

                    indexer.resetOuttakeState();

                    follower.followPath(path10, true);

                    setPathState(10);
                }
                break;


            // =========================================================
            // 10: FINAL RETURN + FINAL OUTTAKE
            // =========================================================
            case 10:

                if (!follower.isBusy()
                        && actionTimer.getElapsedTimeSeconds() > 0.2) {

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

        drive = new DriveSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap, drive);

        buildPaths();

        waitForStart();

        setPathState(0);

        while (opModeIsActive() && !isStopRequested()) {

            follower.update();

            autonomousPathUpdate();

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData(
                    "Heading",
                    Math.toDegrees(follower.getPose().getHeading())
            );

            telemetry.update();
        }
    }
}