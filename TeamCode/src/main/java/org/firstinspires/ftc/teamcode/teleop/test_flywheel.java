package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "test_flywheel", group = "Test")
public class test_flywheel extends OpMode {
    DcMotorEx MO1;
    double tick_rpm = 28;
    double rpm_fly = 3000;
    boolean ldpl = false;
    boolean ldpr = false;
    double kP = 0.0001;
    double kI = 0.00005;
    double kD = 0.000005;
    double kF = 0.00018;
    double lastError = 0;
    double ticksPerSecond = 0;
    double current_fly = 0;
    double error = 0;
    double integral = 0;
    double derivative = 0;
    ElapsedTime dt = new ElapsedTime();
    double targetVelocityTicks = 0;
    double currentVelocityTicks = 0;
    double deltaTime = 0;

    double power = 0;
    public void init() {
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        MO1.setDirection(DcMotor.Direction.REVERSE);
        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MO1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dt.reset();


    }
    public void loop(){
        Outtake2Upd();
        Kp();
        telemetry.addData("Target RPM", rpm_fly);
        telemetry.addData("Current RPM", current_fly);
        telemetry.addData("Integral", integral);
        telemetry.addData("Derivative", derivative);
        telemetry.addData("LT", gamepad1.left_trigger);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.addData("dt", deltaTime);
        telemetry.addData("Target TPS", targetVelocityTicks);
        telemetry.addData("Current TPS", ticksPerSecond);


        telemetry.update();

    }
    public void Kp() {
        boolean dpl = gamepad1.dpad_left;
        boolean dpr = gamepad1.dpad_right;

        if (dpl && !ldpl) {
            kP += 0.00001;
        }

        if (dpr && !ldpr) {
            kP -= 0.00001;
        }

        ldpl = dpl;
        ldpr = dpr;
    }
    public void Outtake2Upd() {
        ticksPerSecond = MO1.getVelocity();
        current_fly = ticksPerSecond * 60.0 / tick_rpm;
        targetVelocityTicks = rpm_fly * tick_rpm / 60.0;
        currentVelocityTicks = MO1.getVelocity();
        if(gamepad1.left_trigger > 0.5) {
            error = targetVelocityTicks - currentVelocityTicks;

            deltaTime = Math.max(dt.seconds(), 1e-6);
            dt.reset();

            integral += error * deltaTime;
            integral = Math.max(-5000, Math.min(5000, integral));
            derivative = (error - lastError) / deltaTime;

            power = error * kP + integral * kI + derivative * kD + + targetVelocityTicks * kF;

            power = Math.max(-1.0, Math.min(1.0, power));


            MO1.setPower(power);
        }
        else {
            MO1.setPower(0);

            integral = 0;
            lastError = 0;
            dt.reset();
        }

        lastError = error;
    }
}