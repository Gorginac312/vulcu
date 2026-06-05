package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.unused.Intake;
import org.firstinspires.ftc.teamcode.unused.Outtake;
import org.firstinspires.ftc.teamcode.unused.ShootActionContinu;
import org.firstinspires.ftc.teamcode.unused.ShootActionContinuGate;
import org.firstinspires.ftc.teamcode.unused.Values;

import java.util.List;

@TeleOp(name = "DriveTulumba_V6", group = "Teleop")
public class DriveTulumba_V6 extends OpMode {

    /* ================= SYSTEMS ================= */
    Intake intake;
    Outtake outtake;
    Values values;

    /* ================= DRIVE ================= */
    DcMotor LB, LF, RB, RF;

    /* ================= OUTTAKE MOTOR ========= */
    DcMotorEx OuttakeContinu;

    /* ================= FLYWHEEL ================= */
    DcMotorEx flywheel;

    /* ================= ELEVATE ================= */
    private DcMotor Elevate;
    private Servo servo;

    private double stepStart;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime elevateTimer = new ElapsedTime();

    private int elevateUpUsed = 0;
    private int elevateDownUsed = 0;
    private int elevateState = 0;
    // 0 = idle, 1 = up, 2 = down

    /* ================= INDEXER ================= */
    boolean lastCircle = false;
    boolean lastSquare = false;

    int k = 1; // intake pose step (1..3)
    int i = 0; // outtake pose step (1..3)

    /* ================= INTAKE FIX ================= */
    private boolean intakeRunning = false;
    private boolean lastIntakeRunning = false;

    /* ================= LIMELIGHT ================= */
    private static final String LIMELIGHT_NAME = "limelight";
    private Limelight3A limelight;

    // Tag IDs
    private static final int TAG_BLUE = 20;
    private static final int TAG_RED  = 24;

    // Selected target tag (-1 = none)
    private int selectedTagId = -1;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;

    /* ================= AUTO HEADING (PD) ================= */
    private boolean autoHeadingEnabled = false; // toggled by gamepad1.square
    private boolean lastG1Square = false;

    private double lastHeadingError = 0.0;
    private double lastHeadingTime  = 0.0;

    // Debug telemetry
    private double dbgTx = Double.NaN;
    private double dbgTxGoal = Double.NaN;
    private double dbgErr = 0.0;
    private double dbgCmd = 0.0;
    private String dbgMode = "MANUAL";

    /* ================= AUTO RPM MODEL ================= */
    private static final double D_CLOSE_M = 1.10;
    private static final double D_MID_M   = 1.70;
    private static final double D_FAR_M   = 2.90;
    private static final double D_ALLOW_4K_M = 3.20;
    private static final int RPM_LIMIT_4K = 4000;
    private static final int DEFAULT_MAX_RPM = 4000;

    /* ===== AutoRPM failsafe ===== */
    private int lastGoodAutoRpm = 0;
    private double lastGoodAutoRpmTime = 0.0;
    private static final double AUTO_RPM_HOLD_TIMEOUT_S = 2.0;

    // AutoRPM telemetry
    private boolean autoRpmUsingHold = false;
    private int autoRpmCommanded = 0;

    // Always computed RPM from distance (even if flywheel is OFF)
    private int computedAutoRpm = 0;

    /* ================= BEAM AUTO INDEX (toggle gp1 CROSS) ================= */
    private boolean autoIndexMode = false;      // toggled by gamepad1 CROSS
    private boolean lastCross = false;

    // ball counter
    private int ballsTaken = 0;
    private static final int BALLS_MAX = 3;

    /* ================= PIPELINE MGMT ================= */
    private static final int PIPE_AUTO_HEADING = 5;
    private int lastPipeline = -999;

    /* ================= SHOOT ACTIONS (TeleOp) ================= */
    private ShootActionContinu shootActionContinu;
    private ShootActionContinuGate shootActionContinuGate;
    private boolean lastG2Cross = false;

    /* ================= AUTO HEADING RANGE LATCH =================
       FAR/CLOSE depends on COMPUTED RPM (not actual flywheel RPM).
       Prevents far/close switching during bursts (RPM dips).
     */
    private static final int RPM_FAR_ENTER = 3400;
    private static final int RPM_FAR_EXIT  = 3200;
    private static final double RANGE_HOLD_AFTER_SHOT_S = 0.90;

    private boolean headingFarLatched = false;
    private double headingRangeHoldUntil = 0.0;

    @Override
    public void init() {

        values = new Values();
        intake = new Intake(hardwareMap, values);
        outtake = new Outtake(hardwareMap);

        flywheel = outtake.MO1;

        OuttakeContinu = hardwareMap.get(DcMotorEx.class, "OuttakeContinu");
        OuttakeContinu.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeContinu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        Elevate = hardwareMap.get(DcMotor.class, "Elevate");
        Elevate.setDirection(DcMotorSimple.Direction.FORWARD);
        Elevate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        k = 0;
        i = 0;
        intake.Indexer.setPosition(values.IntakePose1);

        outtake.ServoLimeLight.setPosition(values.TagServoStartPose);

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(Values.TagPipeline);
        limelight.start();
        lastPipeline = Values.TagPipeline;

        intakeRunning = false;
        lastIntakeRunning = false;

        autoIndexMode = false;
        ballsTaken = 0;

        lastHeadingTime = getRuntime();
        lastHeadingError = 0.0;

        shootActionContinu = new ShootActionContinu();
        shootActionContinu.init(hardwareMap, outtake, intake, values);

        shootActionContinuGate = new ShootActionContinuGate();
        shootActionContinuGate.init(hardwareMap, outtake, intake, values);

        servo = hardwareMap.get(Servo.class, "ServoLimeLight");

        telemetry.addLine("=== CONTROLS (V6) ===");
        telemetry.addLine("GAMEPAD1:");
        telemetry.addLine("  LS drive | RSx turn | LT slow");
        telemetry.addLine("  RT intake | RB reverse intake");
        telemetry.addLine("  CROSS toggle Beam AutoIndex");
        telemetry.addLine("  SQUARE toggle AutoHeading");
        telemetry.addLine("  DPAD_LEFT tag20 BLUE | DPAD_RIGHT tag24 RED");
        telemetry.addLine("  DPAD_UP/DOWN elevate");
        telemetry.addLine("GAMEPAD2:");
        telemetry.addLine("  LT AutoRPM (with hold) | DPAD_UP mid rpm | DPAD_LEFT far rpm");
        telemetry.addLine("  CROSS shoot: CLOSE=continu / FAR=gated");
        telemetry.addLine("  TRIANGLE push servo");
        telemetry.addLine("  SQUARE cycle INTAKE poses | CIRCLE cycle OUTTAKE poses");
        telemetry.update();
    }

    @Override
    public void loop() {

        updateSelectedTag();
        updateAutoHeadingToggle();
        updateAutoIndexToggle();

        int desiredPipeline = autoHeadingEnabled ? PIPE_AUTO_HEADING : Values.TagPipeline;
        if (desiredPipeline != lastPipeline) {
            limelight.pipelineSwitch(desiredPipeline);
            lastPipeline = desiredPipeline;
        }

        updateComputedAutoRpm();
        handleShootActionStartByRange();
        updateHeadingRangeLatch();

        Chassi();
        IntakeMotor();
        BeamAutoIndexUpdate();

        boolean continuBusy = shootActionContinu != null && shootActionContinu.isBusy();
        boolean gateBusy = shootActionContinuGate != null && shootActionContinuGate.isBusy();

        if (continuBusy) {
            shootActionContinu.update();
        } else if (gateBusy) {
            shootActionContinuGate.update();
        } else {
            OuttakeContinuMec();
            Indexer();
            FlywheelControl();
            PushServo();
        }

        servo.setPosition(values.TunPose);

        telemetry.addData("ShootActionContinu", (shootActionContinu != null && shootActionContinu.isBusy()) ? "BUSY" : "IDLE");
        telemetry.addData("ShootActionGated", (shootActionContinuGate != null && shootActionContinuGate.isBusy()) ? "BUSY" : "IDLE");

        telemetry.addData("AUTO HEADING", autoHeadingEnabled ? "ON" : "OFF");
        telemetry.addData("SelectedTag", selectedTagId == -1 ? "NONE" : selectedTagId);
        telemetry.addData("LL pipeline", lastPipeline);

        telemetry.addData("HeadingMode", dbgMode);
        telemetry.addData("tx(deg)", Double.isNaN(dbgTx) ? "NaN" : String.format("%.2f", dbgTx));
        telemetry.addData("txGoal(deg)", Double.isNaN(dbgTxGoal) ? "NaN" : String.format("%.2f", dbgTxGoal));
        telemetry.addData("RangeLatch", headingFarLatched ? "FAR" : "CLOSE");
        telemetry.addData("err(deg)", "%.2f", dbgErr);
        telemetry.addData("rotCmd", "%.3f", dbgCmd);

        telemetry.addLine("---- Indexer ----");
        telemetry.addData("IndexerPos", "%.3f", intake.Indexer.getPosition());
        telemetry.addData("k(IntakeStep)", k);
        telemetry.addData("i(OuttakeStep)", i);

        telemetry.addLine("---- Beam ----");
        telemetry.addData("AutoIndexMode (gp1 CROSS)", autoIndexMode ? "ON" : "OFF");
        telemetry.addData("BeamRaw", intake.getBeamRaw());
        telemetry.addData("ballsTaken", ballsTaken);

        telemetry.addLine("---- Intake ----");
        telemetry.addData("IntakeRunning", intakeRunning);
        telemetry.addData("Intake RPM", "%.0f", intake.getIntakeRPM());

        telemetry.addLine("---- Flywheel ----");
        telemetry.addData("ComputedAutoRPM", computedAutoRpm);
        telemetry.addData("AutoRPM source", autoRpmUsingHold ? "HOLD(lastGood)" : "FRESH(tag)");
        telemetry.addData("Fly Cmd RPM", autoRpmCommanded);
        telemetry.addData("Fly Actual RPM", "%.1f", outtake.getFlyWheelRPM());

        telemetry.update();
    }

    private void updateComputedAutoRpm() {
        computedAutoRpm = 0;

        if (selectedTagId == -1) return;

        FiducialResult tag = getTagById(selectedTagId);
        if (tag == null) return;

        double distM = getDistanceMeters(tag);
        if (Double.isNaN(distM) || distM <= 0) return;

        computedAutoRpm = getAutoTargetRpm(distM);
    }

    private void updateAutoIndexToggle() {
        boolean cross = gamepad1.cross;

        if (cross && !lastCross) {
            autoIndexMode = !autoIndexMode;

            if (autoIndexMode) {
                k = 1;
                i = 1;
                intake.Indexer.setPosition(values.IntakePose1);

                ballsTaken = 0;
                intake.resetBeamForNewBall();
            }
        }
        lastCross = cross;
    }

    private void updateAutoHeadingToggle() {
        boolean sq = gamepad1.square;
        if (sq && !lastG1Square) {
            autoHeadingEnabled = !autoHeadingEnabled;

            lastHeadingTime = getRuntime();
            lastHeadingError = 0.0;
        }
        lastG1Square = sq;
    }

    private void updateSelectedTag() {
        boolean dl = gamepad1.dpad_left;
        boolean dr = gamepad1.dpad_right;

        if (dl && !lastDpadLeft) {
            if (selectedTagId == TAG_BLUE) selectedTagId = -1;
            else selectedTagId = TAG_BLUE;
        }

        if (dr && !lastDpadRight) {
            if (selectedTagId == TAG_RED) selectedTagId = -1;
            else selectedTagId = TAG_RED;
        }

        lastDpadLeft = dl;
        lastDpadRight = dr;
    }

    private void updateHeadingRangeLatch() {
        double now = getRuntime();

        boolean continuBusy = shootActionContinu != null && shootActionContinu.isBusy();
        boolean gateBusy = shootActionContinuGate != null && shootActionContinuGate.isBusy();

        if (continuBusy || gateBusy) {
            headingRangeHoldUntil = Math.max(headingRangeHoldUntil, now + 0.05);
            return;
        }

        if (now < headingRangeHoldUntil) return;

        int rpmRef = (computedAutoRpm > 0) ? computedAutoRpm : autoRpmCommanded;

        if (!headingFarLatched) {
            if (rpmRef >= RPM_FAR_ENTER) headingFarLatched = true;
        } else {
            if (rpmRef <= RPM_FAR_EXIT) headingFarLatched = false;
        }
    }

    private double getDynamicTxGoalDeg() {
        if (selectedTagId == TAG_BLUE) {
            return headingFarLatched ? Values.Heading_TxGoalBlueFarDeg : Values.Heading_TxGoalBlueCloseDeg;
        } else if (selectedTagId == TAG_RED) {
            return headingFarLatched ? Values.Heading_TxGoalRedFarDeg : Values.Heading_TxGoalRedCloseDeg;
        }
        return Values.Heading_TxGoalDeg;
    }

    public void Chassi() {

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;

        double driverYaw = gamepad1.right_stick_x;

        double slowMode = gamepad1.left_trigger > 0.05 ? 0.4 : 1.0;

        double yawCmd = driverYaw;
        dbgMode = "MANUAL";
        dbgTx = Double.NaN;
        dbgTxGoal = Double.NaN;
        dbgErr = 0.0;

        if (autoHeadingEnabled && selectedTagId != -1) {
            FiducialResult tag = getTagById(selectedTagId);

            if (tag != null) {
                dbgMode = "AUTO (tag found)";

                double tx = tag.getTargetXDegrees();
                dbgTx = tx;

                double txGoal = getDynamicTxGoalDeg();
                dbgTxGoal = txGoal;

                double error = txGoal - tx;
                dbgErr = error;

                if (Math.abs(error) < Values.Heading_DeadbandDeg) {
                    yawCmd = 0.0;
                    lastHeadingError = error;
                    lastHeadingTime = getRuntime();
                } else {
                    double pTerm = error * Values.Heading_kP;

                    double now = getRuntime();
                    double dT = now - lastHeadingTime;
                    dT = Math.max(1e-3, dT);

                    double dTerm = ((error - lastHeadingError) / dT) * Values.Heading_kD;

                    double cmd = pTerm + dTerm;
                    cmd = Range.clip(cmd, -Values.Heading_MaxRot, Values.Heading_MaxRot);

                    if (Math.abs(cmd) > 0 && Math.abs(cmd) < Values.Heading_MinRot) {
                        cmd = Math.signum(cmd) * Values.Heading_MinRot;
                    }

                    if (Values.Heading_Invert) cmd = -cmd;

                    yawCmd = cmd;

                    lastHeadingError = error;
                    lastHeadingTime = now;
                }
            } else {
                dbgMode = "AUTO (tag missing -> driver yaw)";
                lastHeadingError = 0.0;
                lastHeadingTime = getRuntime();
            }
        } else {
            lastHeadingError = 0.0;
            lastHeadingTime = getRuntime();
        }

        dbgCmd = yawCmd;

        double ax = axial * slowMode;
        double lat = lateral * slowMode;

        double lfT = ax + lat;
        double rfT = ax - lat;
        double lbT = ax - lat;
        double rbT = ax + lat;

        double maxT = Math.max(Math.abs(lfT),
                Math.max(Math.abs(rfT),
                        Math.max(Math.abs(lbT), Math.abs(rbT))));
        if (maxT > 1.0) {
            lfT /= maxT;
            rfT /= maxT;
            lbT /= maxT;
            rbT /= maxT;
        }

        double yaw = yawCmd * slowMode;
        double headroom = 1.0 - Math.abs(yaw);
        headroom = Math.max(0.0, headroom);

        lfT *= headroom;
        rfT *= headroom;
        lbT *= headroom;
        rbT *= headroom;

        double lf = lfT + yaw;
        double rf = rfT - yaw;
        double lb = lbT + yaw;
        double rb = rbT - yaw;

        lf = Range.clip(lf, -1.0, 1.0);
        rf = Range.clip(rf, -1.0, 1.0);
        lb = Range.clip(lb, -1.0, 1.0);
        rb = Range.clip(rb, -1.0, 1.0);

        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }

    private void ElevateControl() {
        if (elevateState == 0) {
            Elevate.setPower(0.0);

            if (gamepad1.dpad_up && elevateUpUsed == 0) {
                elevateUpUsed = 1;
                elevateState = 1;
                elevateTimer.reset();
            } else if (gamepad1.dpad_down && elevateDownUsed == 0) {
                elevateDownUsed = 1;
                elevateState = 2;
                elevateTimer.reset();
            }
        }

        if (elevateState == 1) {
            if (elevateTimer.milliseconds() < 1000) {
                Elevate.setPower(-1);
            } else {
                Elevate.setPower(0.0);
                elevateState = 0;
            }
        } else if (elevateState == 2) {
            if (elevateTimer.milliseconds() <500) {
                Elevate.setPower(1);
            } else {
                Elevate.setPower(0.0);
                elevateState = 0;
            }
        }
    }

    public void IntakeMotor() {

        boolean forward = gamepad1.right_trigger > 0.05;
        boolean reverse = gamepad1.right_bumper;

        if (reverse) {
            intakeRunning = true;
            setIntakeRPM_Teleop(-values.IntakeRPM);
            return;
        }

        if (forward) {
            intakeRunning = true;
            setIntakeRPM_Teleop(values.IntakeRPM);
            return;
        }

        intakeRunning = false;
        intake.IntakeMotor.setVelocity(0);
    }

    private void BeamAutoIndexUpdate() {

        boolean intakeJustStarted = intakeRunning && !lastIntakeRunning;
        if (intakeJustStarted && autoIndexMode) {
            intake.resetBeamForNewBall();
        }
        lastIntakeRunning = intakeRunning;

        if (!autoIndexMode) return;
        if (!intakeRunning) return;

        if (ballsTaken >= BALLS_MAX) return;

        if (intake.consumeBallCapturedEvent(true)) {

            ballsTaken++;

            if (ballsTaken >= BALLS_MAX) {
                autoIndexMode = false;
                return;
            }

            if (k == 1) {
                intake.Indexer.setPosition(values.IntakePose2);
                k = 2;
            } else if (k == 2) {
                intake.Indexer.setPosition(values.IntakePose3);
                k = 3;
            } else {
                intake.Indexer.setPosition(values.IntakePose1);
                k = 1;
            }
        }
    }

    public void Indexer() {

        if (autoIndexMode) return;

        boolean currentCircle = gamepad2.circle;
        boolean currentSquare = gamepad2.square;

        if (currentSquare && !lastSquare) {
            i = 0;

            k++;
            if (k > 3) k = 1;

            if (k == 1)      intake.Indexer.setPosition(values.IntakePose1);
            else if (k == 2) intake.Indexer.setPosition(values.IntakePose2);
            else             intake.Indexer.setPosition(values.IntakePose3);
        }

        if (currentCircle && !lastCircle) {
            k = 0;

            i++;
            if (i > 3) i = 1;

            if (i == 1)      intake.Indexer.setPosition(values.OuttakePose1);
            else if (i == 2) intake.Indexer.setPosition(values.OuttakePose2);
            else             intake.Indexer.setPosition(values.OuttakePose3);
        }

        lastCircle = currentCircle;
        lastSquare = currentSquare;
    }

    private void FlywheelControl() {

        autoRpmUsingHold = false;
        autoRpmCommanded = 0;

        boolean autoEnable = (gamepad2.left_trigger > 0.05);

        if (autoEnable) {
            int targetRpm = 0;

            if (selectedTagId != -1) {
                FiducialResult tag = getTagById(selectedTagId);
                if (tag != null) {
                    double distM = getDistanceMeters(tag);
                    targetRpm = getAutoTargetRpm(distM);

                    if (targetRpm > 0) {
                        lastGoodAutoRpm = targetRpm;
                        lastGoodAutoRpmTime = getRuntime();
                    }
                }
            }

            if (targetRpm <= 0 && lastGoodAutoRpm > 0) {
                double age = getRuntime() - lastGoodAutoRpmTime;
                if (AUTO_RPM_HOLD_TIMEOUT_S <= 0.0 || age <= AUTO_RPM_HOLD_TIMEOUT_S) {
                    targetRpm = lastGoodAutoRpm;
                    autoRpmUsingHold = true;
                }
            }

            autoRpmCommanded = targetRpm;

            if (targetRpm <= 0) flywheel.setVelocity(0);
            else setFlywheelRPM(targetRpm);

            return;
        }

        if (gamepad1.left_trigger>0.05) {
            autoRpmCommanded = Values.MidShootRPM;
            setFlywheelRPM(autoRpmCommanded);
        } else if (gamepad2.dpad_left) {
            autoRpmCommanded = Values.FarShootRPM;
            setFlywheelRPM(autoRpmCommanded);
        } else {
            autoRpmCommanded = 0;
            flywheel.setVelocity(0);
        }
    }

    private int getAutoTargetRpm(double distM) {
        if (Double.isNaN(distM) || distM <= 0) return 0;

        final int rpmClose = Values.CloseShootRPM;
        final int rpmMid   = Values.MidShootRPM;
        final int rpmFar   = Values.FarShootRPM;

        double rpm;

        if (distM <= D_CLOSE_M) {
            rpm = rpmClose;
        } else if (distM <= D_MID_M) {
            rpm = lerp(D_CLOSE_M, rpmClose, D_MID_M, rpmMid, distM);
        } else if (distM <= D_FAR_M) {
            rpm = lerp(D_MID_M, rpmMid, D_FAR_M, rpmFar, distM);
        } else {
            rpm = rpmFar;
        }

        int maxAllowed = (distM > D_ALLOW_4K_M) ? RPM_LIMIT_4K : DEFAULT_MAX_RPM;
        rpm = Range.clip(rpm, 0, maxAllowed);
        return (int) Math.round(rpm);
    }

    private double lerp(double x0, double y0, double x1, double y1, double x) {
        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    public void PushServo() {
        if (autoIndexMode) {
            outtake.PushServo.setPosition(0.2);
        } else if (gamepad2.triangle) {
            outtake.PushServo.setPosition(values.PushEndPose);
        } else {
            outtake.PushServo.setPosition(values.PushStartPose);
        }
    }

    /* ================= SHOOT ACTION START (GAMEPAD 2 CROSS) ================= */
    private void handleShootActionStartByRange() {
        boolean cross = gamepad1.triangle;

        boolean continuBusy = shootActionContinu != null && shootActionContinu.isBusy();
        boolean gateBusy = shootActionContinuGate != null && shootActionContinuGate.isBusy();
        boolean anyBusy = continuBusy || gateBusy;

        if (cross && !lastG2Cross && !anyBusy) {

            int rpmForShots = (computedAutoRpm > 0)
                    ? computedAutoRpm
                    : ((autoRpmCommanded > 0) ? autoRpmCommanded : Values.MidShootRPM);

            boolean useFarInstance = rpmForShots >= RPM_FAR_ENTER;

            headingFarLatched = useFarInstance;
            headingRangeHoldUntil = getRuntime() + RANGE_HOLD_AFTER_SHOT_S;

            if (useFarInstance) {
                shootActionContinuGate.fireShots(rpmForShots, 3);
            } else {
                shootActionContinu.fireShots(rpmForShots, 3);
            }
        }

        lastG2Cross = cross;
    }

    private FiducialResult getTagById(int id) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        for (FiducialResult f : tags) {
            if (f.getFiducialId() == id) return f;
        }
        return null;
    }

    private double getDistanceMeters(FiducialResult tag) {
        Pose3D pose = tag.getRobotPoseTargetSpace();
        if (pose == null) return Double.NaN;

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double z = pose.getPosition().z;

        return Math.sqrt(x * x + y * y + z * z);
    }

    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = rpm * Values.FLYWHEEL_TICKS_PER_REV / 60.0;
        flywheel.setVelocity(ticksPerSecond);
    }

    public void OuttakeContinuMec() {
        if (gamepad2.right_trigger > 0) {
            OuttakeContinu.setPower(1);
        } else {
            OuttakeContinu.setPower(0);
        }
    }

    public void setIntakeRPM_Teleop(double rpm) {
        double ticksPerSecond = rpm * Values.FLYWHEEL_TICKS_PER_REV / 60.0;
        intake.IntakeMotor.setVelocity(ticksPerSecond);
    }

    private boolean elapsedMs(double ms) {
        return (timer.milliseconds() - stepStart) >= ms;
    }
}
