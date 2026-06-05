package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Outtake {

    public DcMotorEx MO1;          // Flywheel (PID + velocity)
    public ServoImplEx PushServo;
    public ServoImplEx ServoLimeLight;

    Values values;

    public Outtake(HardwareMap hardwareMap) {

        values = new Values();

        MO1 = hardwareMap.get(DcMotorEx.class, "MO1");

        PushServo = hardwareMap.get(ServoImplEx.class, "PushServo");
        ServoLimeLight = hardwareMap.get(ServoImplEx.class, "ServoLimeLight");

        // --- Encoder ---
        MO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MO1.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- PIDF ---
        setPIDF( values.P , values.F);
    }

    /* ---------------- PIDF ---------------- */

    public void setPIDF(double p, double f) {
        PIDFCoefficients pidf = new PIDFCoefficients(
                p,
                0,
                0,
                f
        );

        MO1.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
    }

    /* ---------------- FLYWHEEL API ---------------- */

    /** Set flywheel speed in RPM */
    public void setFlyWheelRPM(double rpm) {
        double ticksPerSecond =
                rpm * Values.FLYWHEEL_TICKS_PER_REV / 60.0;
        MO1.setVelocity(ticksPerSecond);
    }

    public void stopFlywheel() {
        MO1.setVelocity(0);
    }

    public double getFlyWheelRPM() {
        return MO1.getVelocity() * 60.0
                / Values.FLYWHEEL_TICKS_PER_REV;
    }
}
