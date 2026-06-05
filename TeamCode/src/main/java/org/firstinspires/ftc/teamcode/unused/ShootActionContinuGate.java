package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootActionContinuGate {

    public Values values;
    public Outtake outtake;
    public Intake intake;

    private final ElapsedTime timer = new ElapsedTime();
    private double stepStart = 0;

    // ===== RPM GATING =====
    private double targetRPM = 0.0;

    // Tune these
    private static final double RPM_TOLERANCE = 60;        // allowed error from target
    private static final double RPM_STABLE_TIME_MS = 30.0;   // must stay in range this long
    private static final double RPM_GATE_TIMEOUT_MS = 500; // fail-safe so it doesn't hang forever

    private double rpmStableStart = -1;

    private enum ShootStep {
        PRE_DELAY,            // used for the dynamic "move-to-pose1" wait
        SET_INDEXER,
        WAIT_INDEXER,
        WAIT_FLYWHEEL_READY,  // NEW
        WAIT_PUSH_UP,
        WAIT_PUSH_DOWN,
        DONE
    }

    private ShootStep shootStep = ShootStep.DONE;

    private int shootBallIndex = 0;
    private int totalShots = 0;

    // ===== Dynamic start delay (distance-based) =====
    // IMPORTANT: We DO NOT "wait first then move".
    // We MOVE to OuttakePose1 immediately, then WAIT the delay.
    private static final double POSE_TOL = 0.01; // servo position tolerance

    // Outtake pose delays (ms) when starting a 3-shot burst and needing to go to OuttakePose1
    private static final double DELAY_FROM_OUTTAKE_POSE1_MS = 0.0;
    private static final double DELAY_FROM_OUTTAKE_POSE2_MS = 250.0;
    private static final double DELAY_FROM_OUTTAKE_POSE3_MS = 500.0;

    // Intake pose delays (ms) -> going to OuttakePose1
    private static final double DELAY_FROM_INTAKE_POSE1_MS = 50.0;
    private static final double DELAY_FROM_INTAKE_POSE2_MS = 300.0;
    private static final double DELAY_FROM_INTAKE_POSE3_MS = 550.0;

    private double currentPreDelayMs = 0.0;

    public void init(HardwareMap hw, Outtake outtakeSys, Intake intakeSys, Values vals) {
        outtake = outtakeSys;
        intake = intakeSys;
        values = vals;

        shootStep = ShootStep.DONE;
        shootBallIndex = 0;
        totalShots = 0;
        targetRPM = 0.0;
        rpmStableStart = -1;

        outtake.setFlyWheelRPM(0);
        outtake.PushServo.setPosition(values.PushStartPose);
    }

    public void fireShots(double rpm, int numShots) {
        if (shootStep != ShootStep.DONE) return;

        totalShots = numShots;
        shootBallIndex = 0;
        targetRPM = rpm;
        rpmStableStart = -1;

        // Start flywheel + ensure push is reset
        outtake.setFlyWheelRPM(rpm);
        outtake.PushServo.setPosition(values.PushStartPose);

        // ===== START BEHAVIOR FOR 3 SHOTS =====
        // Move indexer to OuttakePose1 immediately, then wait based on CURRENT servo pose.
        currentPreDelayMs = 0.0;

        if (numShots == 3) {
            double currentPos = intake.Indexer.getPosition();
            currentPreDelayMs = computeDelayToOuttakePose1Ms(currentPos);

            // Move right now
            intake.Indexer.setPosition(values.OuttakePose1);

            timer.reset();
            stepStart = timer.milliseconds();

            // After the delay, go to flywheel gate
            shootStep = (currentPreDelayMs > 0.0) ? ShootStep.PRE_DELAY : ShootStep.WAIT_INDEXER;
            return;
        }

        // For non-3-shot calls, keep normal flow
        timer.reset();
        stepStart = timer.milliseconds();
        shootStep = ShootStep.SET_INDEXER;
    }

    public void update() {
        switch (shootStep) {

            case PRE_DELAY:
                if (elapsedMs(currentPreDelayMs)) {
                    stepStart = timer.milliseconds();
                    shootStep = ShootStep.WAIT_INDEXER;
                }
                break;

            case SET_INDEXER:
                setIndexerPose(shootBallIndex);
                stepStart = timer.milliseconds();
                shootStep = ShootStep.WAIT_INDEXER;
                break;

            case WAIT_INDEXER:
                if (elapsedMs(values.INDEXER_SETTLE_TIME)) {
                    rpmStableStart = -1;
                    stepStart = timer.milliseconds();
                    shootStep = ShootStep.WAIT_FLYWHEEL_READY;
                }
                break;

            case WAIT_FLYWHEEL_READY:
                if (isFlywheelReady()) {
                    intake.OuttakeContinu.setPower(1);
                    stepStart = timer.milliseconds();
                    shootStep = ShootStep.WAIT_PUSH_UP;
                } else if (elapsedMs(RPM_GATE_TIMEOUT_MS)) {
                    // fail-safe: still fire if gate takes too long
                    intake.OuttakeContinu.setPower(1);
                    stepStart = timer.milliseconds();
                    shootStep = ShootStep.WAIT_PUSH_UP;
                }
                break;

            case WAIT_PUSH_UP:
                if (elapsedMs(120)) {
                    intake.OuttakeContinu.setPower(0);
                    stepStart = timer.milliseconds();
                    shootStep = ShootStep.WAIT_PUSH_DOWN;
                }
                break;

            case WAIT_PUSH_DOWN:
                shootBallIndex++;

                if (shootBallIndex >= totalShots) {
                    outtake.setFlyWheelRPM(0);
                    targetRPM = 0.0;
                    shootStep = ShootStep.DONE;
                } else {
                    shootStep = ShootStep.SET_INDEXER;
                }
                break;

            case DONE:
            default:
                break;
        }
    }

    private void setIndexerPose(int index) {
        // 3-shot order: Pose1, Pose2, Pose3
        if (index == 0) {
            intake.Indexer.setPosition(values.OuttakePose1);
        } else if (index == 1) {
            intake.Indexer.setPosition(values.OuttakePose2);
        } else {
            intake.Indexer.setPosition(values.OuttakePose3);
        }
    }

    // ===== RPM GATE =====
    private boolean isFlywheelReady() {
        double currentRPM = getCurrentFlywheelRPM();

        if (Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE) {
            if (rpmStableStart < 0) {
                rpmStableStart = timer.milliseconds();
            }
            return (timer.milliseconds() - rpmStableStart) >= RPM_STABLE_TIME_MS;
        } else {
            rpmStableStart = -1;
            return false;
        }
    }

    // IMPORTANT:
    // Change this to match YOUR Outtake RPM getter.
    // Examples:
    // return outtake.getFlyWheelRPM();
    // return outtake.getCurrentRPM();
    // return outtake.flyWheelMotor.getVelocity() * SOME_CONVERSION;
    private double getCurrentFlywheelRPM() {
        return outtake.getFlyWheelRPM(); // <-- replace only if your method name is different
    }

    // ===== Delay mapping you requested =====
    // Outtake: Pose1=0, Pose2=250, Pose3=500
    // Intake:  Pose1=50, Pose2=300, Pose3=550
    private double computeDelayToOuttakePose1Ms(double pos) {

        if (near(pos, values.OuttakePose1)) return DELAY_FROM_OUTTAKE_POSE1_MS;
        if (near(pos, values.OuttakePose2)) return DELAY_FROM_OUTTAKE_POSE2_MS;
        if (near(pos, values.OuttakePose3)) return DELAY_FROM_OUTTAKE_POSE3_MS;

        if (near(pos, values.IntakePose1)) return DELAY_FROM_INTAKE_POSE1_MS;
        if (near(pos, values.IntakePose2)) return DELAY_FROM_INTAKE_POSE2_MS;
        if (near(pos, values.IntakePose3)) return DELAY_FROM_INTAKE_POSE3_MS;

        double dO1 = Math.abs(pos - values.OuttakePose1);
        double dO2 = Math.abs(pos - values.OuttakePose2);
        double dO3 = Math.abs(pos - values.OuttakePose3);
        double dI1 = Math.abs(pos - values.IntakePose1);
        double dI2 = Math.abs(pos - values.IntakePose2);
        double dI3 = Math.abs(pos - values.IntakePose3);

        double best = dO1;
        double delay = DELAY_FROM_OUTTAKE_POSE1_MS;

        if (dO2 < best) { best = dO2; delay = DELAY_FROM_OUTTAKE_POSE2_MS; }
        if (dO3 < best) { best = dO3; delay = DELAY_FROM_OUTTAKE_POSE3_MS; }
        if (dI1 < best) { best = dI1; delay = DELAY_FROM_INTAKE_POSE1_MS; }
        if (dI2 < best) { best = dI2; delay = DELAY_FROM_INTAKE_POSE2_MS; }
        if (dI3 < best) { best = dI3; delay = DELAY_FROM_INTAKE_POSE3_MS; }

        return delay;
    }

    private boolean near(double a, double b) {
        return Math.abs(a - b) <= POSE_TOL;
    }

    private boolean elapsedMs(double ms) {
        return (timer.milliseconds() - stepStart) >= ms;
    }

    public boolean isBusy() {
        return shootStep != ShootStep.DONE;
    }
}
