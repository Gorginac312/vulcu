package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    public DcMotorEx IntakeMotor;   // 6000 RPM motor
    public ServoImplEx Indexer;
    public DcMotorEx OuttakeContinu;

    public Values values;

    // ================= BEAM SENSOR (AUTO + TELEOP) =================
    private RevColorSensorV3 Beam;
    private boolean beamEnabled = false;

    private final ElapsedTime beamCooldown = new ElapsedTime();
    private final ElapsedTime beamRearmTimer = new ElapsedTime();

    private int beamPresentCount = 0;
    private boolean beamArmed = true;

    public Intake(HardwareMap hw, Values vals) {
        values = vals;

        IntakeMotor = hw.get(DcMotorEx.class, "IntakeMotor");
        Indexer = hw.get(ServoImplEx.class, "Indexer");
        OuttakeContinu= hw.get(DcMotorEx.class,"OuttakeContinu");
        OuttakeContinu.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeContinu.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Encoder setup
        IntakeMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(26,0,0,17);
        IntakeMotor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER,pidfCoefficients);

        // Beam init (optional)
        try {
            Beam = hw.get(RevColorSensorV3.class, "Beam");
            beamEnabled = true;
        } catch (Exception e) {
            beamEnabled = false;
        }

        resetBeamForNewBall();
    }

    public void setIntakeRPM(double rpm) {
        double ticksPerSecond = rpm * Values.INTAKE_TICKS_PER_REV / 60.0;
        IntakeMotor.setVelocity(ticksPerSecond);
    }

    public void stopIntake() {
        IntakeMotor.setVelocity(0);
    }

    public double getIntakeRPM() {
        return IntakeMotor.getVelocity() * 60.0 / Values.INTAKE_TICKS_PER_REV;
    }

    // ================= BEAM HELPERS =================

    public int getBeamRaw() {
        if (!beamEnabled || Beam == null) return -1;
        return Beam.rawOptical();
    }

    public void resetBeamForNewBall() {
        beamPresentCount = 0;
        beamArmed = true;
        beamCooldown.reset();
        beamRearmTimer.reset();
    }

    /** Ball present only if MIN <= raw <= MAX (prevents robot-part reflections). */
    private boolean beamBallPresentNow() {
        if (!beamEnabled || Beam == null) return false;

        int raw = Beam.rawOptical();
        return (raw >= Values.BEAM_RAW_MIN) && (raw <= Values.BEAM_RAW_MAX);
    }

    /** Stable detection (N loops). */
    private boolean beamBallPresentStable() {
        boolean presentNow = beamBallPresentNow();

        if (presentNow) beamPresentCount++;
        else beamPresentCount = 0;

        return beamPresentCount >= Values.BEAM_PRESENT_LOOPS;
    }

    /**
     * Returns true ONCE when a ball is detected.
     * Uses:
     * - stability loops
     * - cooldown
     * - rearm timer (so it can still trigger if packed/full)
     *
     * NOTE: we still require intakeRunning to avoid false triggers while idle.
     */
    public boolean consumeBallCapturedEvent(boolean intakeRunning) {
        if (!beamEnabled || Beam == null) return false;
        if (!intakeRunning) return false;

        boolean stable = beamBallPresentStable();

        // RE-ARM:
        // 1) clears -> armed again
        // 2) or still present for BEAM_REARM_S -> armed again (packed/full)
        if (!stable) {
            beamArmed = true;
        } else if (!beamArmed && beamRearmTimer.seconds() >= Values.BEAM_REARM_S) {
            beamArmed = true;
        }

        boolean cooledDown = beamCooldown.seconds() >= Values.BEAM_COOLDOWN_S;

        if (stable && beamArmed && cooledDown) {
            beamArmed = false;
            beamCooldown.reset();
            beamRearmTimer.reset();
            beamPresentCount = 0; // reduce rapid repeats
            return true;
        }

        return false;
    }
}
