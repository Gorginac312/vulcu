package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.DriveSub;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSub;
import org.firstinspires.ftc.teamcode.SubSystems.LimelightSub;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSub;
import org.firstinspires.ftc.teamcode.SubSystems.ValuesSub;


@TeleOp (name = "TeleopMain", group = "Test")
public class TeleopMain extends OpMode {

    //SUBSYSTEMS//
    DriveSub drive;
    IntakeSub intake;
    OuttakeSub outtake;
    IndexerSub indexer;
    LimelightSub limelight;

    //MOTOARE//
    DcMotor Elevate;//elevate, nefolosit//
    double ticksPerSecondIntake = 0;
    double ticksPerSecondOuttake = 0;

    //LIMELIGHT
    double FinalLateral;
    double FinalYaw;
    double FinalAxial;



    @Override
    public void init() {

        Elevate = hardwareMap.get(DcMotor.class , "Elevate");
        drive = new DriveSub(hardwareMap);
        intake = new IntakeSub(hardwareMap);
        outtake = new OuttakeSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap);
        limelight = new LimelightSub(hardwareMap);


        outtake.setPIDF();
        intake.setPIDF();



    }


    public void loop() {
        double CorrectionLateral = limelight.getHeadingCorrection(gamepad1.right_stick_button);
        double CorrectionYaw = limelight.getYawCorrection(gamepad1.right_stick_button);
        double CorrectionAxial = limelight.getTargetCorrection(gamepad1.right_stick_button);


        if (gamepad1.right_stick_button) {
            FinalLateral = -CorrectionLateral;
            FinalYaw = CorrectionYaw;
            FinalAxial = -CorrectionAxial;
        }
        else {
            FinalLateral = gamepad1.left_stick_x;
            FinalYaw = gamepad1.right_stick_x;
            FinalAxial = -gamepad1.left_stick_y;
        }

            drive.drive(
                    FinalAxial,
                    FinalLateral,
                    FinalYaw,
                    gamepad1.left_bumper ? 0.4 : 1.0);


            indexer.FullReset();
            indexer.autoindex(gamepad1.cross);
            indexer.IndexManual(gamepad1.y,
                    gamepad1.b);
            indexer.SensorToggle(gamepad1.left_stick_button);
            if (indexer.getSensorEnabled()) indexer.Sensor();

            intake.update(
                    gamepad1.left_trigger > 0.05,
                    gamepad1.dpad_right,
                    gamepad1.dpad_left);
            if (indexer.GetIndexState() < 1){outtake.update2(gamepad1.x);}
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
            if(limelight.getresult() != null) telemetry.addData("apriltag", limelight.getresult().isValid());
            else telemetry.addData("apriltag", null);
            telemetry.addData("yaw", limelight.getyaw());
            telemetry.addData("tx" , limelight.gettx());
            telemetry.addData("HeadingCorrection" , CorrectionLateral);
            telemetry.addData("YawCorrection" , CorrectionYaw);
            telemetry.update();
        return CorrectionLateral;
    }


    }