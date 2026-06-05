package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "test_flywheel", group = "Test")
public class test_flywheel extends OpMode {
    DcMotorEx MO1;
    double tick_rpm = 28;
    double rpm_fly = 3000;
    boolean ldpu = false;
    boolean ldpd = false;
    boolean ldpl = false;
    boolean ldpr = false;
    double kP = 0.0001;
    double kI = 0.00005;
    double kD = 0.000005;
    double kF = 0.00018;
    double lastError = 0;
    double integral = 0;
    double ticksPerSecond = 0;
    double current_fly = 0;
    public void init() {
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        MO1.setDirection(DcMotor.Direction.REVERSE);
        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void loop(){
        Outtake2();
        Outtake2Upd();
        Kp();
        ticksPerSecond = MO1.getVelocity();
        current_fly = (ticksPerSecond * 60) / tick_rpm;
        telemetry.addData("Target RPM", rpm_fly);
        telemetry.addData("Current RPM", current_fly);
        telemetry.addData("kP" , kP);

        telemetry.update();

    }
    public void Kp() {
        boolean dpl = gamepad1.dpad_left;
        boolean dpr = gamepad1.dpad_right;

        if (dpl && !ldpl) {
            kP += 0.001;
        }

        if (dpr && !ldpr) {
            kP -= 0.001;
        }

        ldpl = dpl;
        ldpr = dpr;
    }
    public void Outtake2() {
        boolean dpu = gamepad1.dpad_up;
        boolean dpd = gamepad1.dpad_down;
        double velocity = (rpm_fly * tick_rpm)/60;
        if(gamepad1.right_trigger > 0.5)
            MO1.setVelocity(velocity);
        else {
            MO1.setVelocity(0.0);
        }
        if (dpu != ldpu) {
            rpm_fly += 100;
        }
        ldpu = dpu;

        if (ldpd != dpd) {
            rpm_fly -= 100;
        }
        ldpd = dpd;
    }
    public void Outtake2Upd() {
        double error = 0;
        double integral = 0;
        double derivative = 0;

        double power = 0;


        telemetry.addData("LT", gamepad1.left_trigger);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);

        if(gamepad1.left_trigger > 0.5) {
            error = rpm_fly - current_fly;
            integral += error;
            derivative = error - lastError;

            power = error * kP + integral * kI + derivative * kD + rpm_fly * kF;

            power = Math.max(-1.0, Math.min(1.0, power));


            MO1.setPower(power);
        }
        else  {
            MO1.setPower(0.0);
        }

        lastError = error;
    }
}