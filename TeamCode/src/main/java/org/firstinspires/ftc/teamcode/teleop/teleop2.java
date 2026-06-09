package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ValuesSub;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "teleop2", group = "Test")
public class teleop2 extends OpMode {
    //LA FEL CA TELEOP1 DAR LA AUTO INDEX ARE OUTTAKE CU ELASTICA CONSTANT,PENTRU EXPLICATII LA TOT VEZI TELEOP1//
    DcMotor LB;
    DcMotor RB;
    DcMotor LF;
    DcMotor RF;
    DcMotorEx IntakeMotor;
    DcMotor OuttakeContinu;
    DcMotorEx MO1;
    DcMotor Elevate;
    Servo Indexer;
    RevColorSensorV3 Beam;
    boolean lastY = false;
    boolean lastB = false;
    int i = 0;
    int o = 0;
    double tick_rpm = 28;
    double rpm_fly = 2500;
    double rpm_far = 3600;
    double rpm_close = 3000;
    double current_rpm = 0;
    double IntakeVelocity = 0;
    double intake_rpm = 0;
    boolean ldpu = false;
    boolean ldpd = false;
    boolean ldpl = false;
    boolean ldpr = false;
    boolean ljoyl = false;
    double slowMode = 0.4;
    boolean objectDetected = false;
    boolean sensorEnabled = false;
    int balls = 0;
    boolean full = false;
    double velocity = 0;
    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime IndexTime = new ElapsedTime();
    ElapsedTime OutTime = new ElapsedTime();
    boolean cross = false;
    boolean lcross = false;
    int IndexState = 0;
    int OutState = 0;
    @Override
    public void init() {

        Beam = hardwareMap.get(RevColorSensorV3.class , "Beam");
        LB = hardwareMap.get(DcMotor.class , "LB");
        RB = hardwareMap.get(DcMotor.class , "RB");
        LF = hardwareMap.get(DcMotor.class , "LF");
        RF = hardwareMap.get(DcMotor.class , "RF");
        IntakeMotor = hardwareMap.get(DcMotorEx.class , "IntakeMotor");
        OuttakeContinu = hardwareMap.get(DcMotor.class , "OuttakeContinu");
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        Elevate = hardwareMap.get(DcMotor.class , "Elevate");
        Indexer = hardwareMap.get(Servo.class , "Indexer");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        MO1.setDirection(DcMotor.Direction.REVERSE);

        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Indexer.setPosition(0.0);
        MO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MO1.setVelocityPIDFCoefficients(
                350.0,
                0.0,
                100.0,
                10.0
        );
        IntakeMotor.setVelocityPIDFCoefficients(
                26.0,
                0.0,
                0.0,
                17.0
        );



    }


    public void loop(){

        Chassis();
        Intake();
        Outtake();
        Outtake2();
        Indexer();
        SensorToggle();
        autoindex();
        double ticksPerSecond = MO1.getVelocity();
        double ticksPerSecondIntake = IntakeMotor.getVelocity();
        velocity = (rpm_fly * tick_rpm)/60;
        current_rpm = (ticksPerSecond * 60) / tick_rpm;
        IntakeVelocity = (ticksPerSecondIntake * 60)/tick_rpm;
        telemetry.addData("Target RPM", rpm_fly);
        telemetry.addData("Current RPM", current_rpm);
        telemetry.addData("intake rpm", IntakeVelocity);
        telemetry.addData("intake target" , intake_rpm);
        telemetry.addData("sensorEnabled", sensorEnabled);
        telemetry.addData("full", full);
        telemetry.addData("balls", balls);

        telemetry.update();
        if(full && fullTimer.seconds() > 1.0) {
            full = false;
            balls = 0;
        }



    }
    public void autoindex() {
        cross = gamepad1.cross;
        if(cross && !lcross && IndexState == 0) {
            Indexer.setPosition(ValuesSub.outtakepos1);
            IndexTime.reset();
            IndexState = 1;
        }
        if(IndexState != 0) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
        }
        if(IndexState == 1 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos2);
            IndexTime.reset();
            IndexState = 2;
        }
        if(IndexState == 2 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos3);
            IndexTime.reset();
            IndexState = 3;
        }
        if(IndexState == 3 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(0.0);
            IndexState = 0;
        }

        lcross = cross;
    }  public void SensorToggle() {
        boolean joyl = gamepad1.left_stick_button;
        if(full) {
            sensorEnabled = false;
        }
        if(joyl && !ljoyl && !full) {
            sensorEnabled = !sensorEnabled;
        }
        ljoyl = joyl;
        if(sensorEnabled) {
            Sensor();
        }
    }
    public void updateIndexer() {

        if(o == 1)      Indexer.setPosition(ValuesSub.intakepos1);
        else if(o == 2) Indexer.setPosition(ValuesSub.intakepos2);
        else            Indexer.setPosition(ValuesSub.intakepos3);
    }
    public void Sensor() {
        double d = Beam.getDistance(DistanceUnit.CM);
        boolean isClose = (d < 2);
        if(isClose && !objectDetected) {
            o++;
            if(o > 3) o = 1;
            updateIndexer();
            balls++;
            if(balls >= 3){
                sensorEnabled = false;
                full = true;
            }
        }
        objectDetected = isClose;
    }
    public void Indexer() {

        boolean currentY = gamepad1.y;
        boolean currentB = gamepad1.b;
        if (currentY && !lastY) {
            i = 0;

            o++;
            if(o > 3) o = 1;

            if(o == 1)      Indexer.setPosition(ValuesSub.intakepos1);
            else if(o == 2) Indexer.setPosition(ValuesSub.intakepos2);
            else            Indexer.setPosition(ValuesSub.intakepos3);
        }

        if(currentB && !lastB) {
            o = 0;

            i++;
            if(i > 3) i = 1;

            if(i == 1)      Indexer.setPosition(ValuesSub.outtakepos1);
            else if(i == 2) Indexer.setPosition(ValuesSub.outtakepos2);
            else            Indexer.setPosition(ValuesSub.outtakepos3);
        }

        lastY = currentY;
        lastB = currentB;
    }

    public void Outtake2() {
        boolean dpu = gamepad1.dpad_up;
        boolean dpd = gamepad1.dpad_down;
        if (gamepad1.right_trigger >= 0.5) {
            MO1.setVelocity(velocity);
        } else {
            MO1.setVelocity(0.0);
        }
        if (dpu && !ldpu) {
            rpm_fly = rpm_far;
        }
        ldpu = dpu;

        if (dpd && !ldpd) {
            rpm_fly = rpm_close;
        }
        ldpd = dpd;
    }

    public void Outtake() {

        if (gamepad1.x) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
        } else {
            OuttakeContinu.setPower(0);
        }
    }

    public void Intake() {
        boolean dpl = gamepad1.dpad_left;
        boolean dpr = gamepad1.dpad_right;
        double IntakeVelocity = (intake_rpm * tick_rpm)/60;
        if(gamepad1.left_trigger > 0.5) {
            IntakeMotor.setVelocity(ValuesSub.targetINT);
        }
        else {
            IntakeMotor.setVelocity(0.0);
        }
        if (dpl && !ldpl) {
            intake_rpm -= 100;
        }
        ldpl = dpl;

        if (dpr && !ldpr) {
            intake_rpm += 100;
        }
        ldpr = dpr;
    }
    public void Chassis() {

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        if(gamepad1.left_bumper) {
            slowMode = 0.4;
        }    else {
            slowMode = 1.0;
        }




        double lf = (axial + lateral + yaw) * slowMode;
        double rf = (axial - lateral - yaw) * slowMode;
        double lb = (axial - lateral + yaw) * slowMode;
        double rb = (axial + lateral - yaw) * slowMode;
        double max = Math.max(Math.abs(lf),
                Math.max(Math.abs(rf),
                        Math.max(Math.abs(lb), Math.abs(rb))));

        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }



}
