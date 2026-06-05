package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.DriveSub;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSub;
import org.firstinspires.ftc.teamcode.SubSystems.ValuesSub;

import com.qualcomm.hardware.rev.RevColorSensorV3;


@TeleOp (name = "Teleop1", group = "Test")
public class Teleop1 extends OpMode {

    //SUBSYSTEMS//
    DriveSub drive;
    IntakeSub intake;
    OuttakeSub outtake;
    IndexerSub indexer;

    //MOTOARE//
    DcMotor Elevate;//elevate, nefolosit//
    Servo Indexer;//indexer//
    RevColorSensorV3 Beam;//senzor distanta/culoare//
    double ticksPerSecondIntake = 0;
    double ticksPerSecondOuttake = 0;


    @Override
    public void init() {

        //DECLARATII//

        Beam = hardwareMap.get(RevColorSensorV3.class , "Beam");
        Elevate = hardwareMap.get(DcMotor.class , "Elevate");
        Indexer = hardwareMap.get(Servo.class , "Indexer");
        drive = new DriveSub(hardwareMap);
        intake = new IntakeSub(hardwareMap);
        outtake = new OuttakeSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap);


        //pozitie indexer basic//
        Indexer.setPosition(0.0);


        //subsystems
        outtake.setPIDF();
        intake.setPIDF();



    }


    public void loop() {
        indexer.FullReset();
        indexer.autoindex(gamepad1.cross);
        indexer.IndexManual(gamepad1.y,
                gamepad1.b);
        indexer.SensorToggle(gamepad1.left_stick_button);
        if(indexer.getSensorEnabled())indexer.Sensor();

        drive.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_bumper ? 0.4 : 1.0);

        intake.update(
                gamepad1.left_trigger > 0.05,
                gamepad1.dpad_right,
                gamepad1.dpad_left);

        outtake.update2(gamepad1.x);
        outtake.update(
                gamepad1.right_trigger > 0.05,
                gamepad1.dpad_down,
                gamepad1.dpad_up);


        ticksPerSecondIntake = intake.getMotor().getVelocity();
        ticksPerSecondOuttake = outtake.getMotor().getVelocity();
        double current_intake = (ticksPerSecondIntake * 60) / ValuesSub.TicksPerRevINT;
        double current_outtake = (ticksPerSecondOuttake * 60) / ValuesSub.TicksPerRevOUT;
        //TELEMETRY(AFISAT PE CONSOLA//

        telemetry.addData("Target RPM", ValuesSub.targetOUT);
        telemetry.addData("Current RPM", current_outtake);
        telemetry.addData("intake target", ValuesSub.targetINT);
        telemetry.addData("intake rpm", current_intake);
        telemetry.addData("sensorEnabled", indexer.getSensorEnabled());
        telemetry.addData("full", indexer.getfull());
        telemetry.addData("balls", indexer.getballs());
        telemetry.update();
    }



}