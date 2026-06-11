package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "test_indexer", group = "Test")

public class test_indexer extends OpMode {
    Servo Indexer;
    boolean lastY = false;
    boolean lastB = false;
    int i = 0;//intake//
    int o = 0;//outtake//
    //pozitii intake indexer//
    double int1 = 0.238;
    double int2 = 0.62;
    double int3 = 1;
    //pozitii outtake indexer//
    double out1 = 0.06;
    double out2 = 0.436;
    double out3 = 0.821;
    double INT = 0.0;
    double OUT = 0.0;
    boolean ldpl = false;
    boolean ldpr = false;
    boolean ldpd = false;
    boolean ldpu = false;
    public void init() {
        Indexer = hardwareMap.get(Servo.class , "Indexer");
        Indexer.setPosition(0.0);
    }

    @Override
    public void loop() {
        UPD();
        IndexPose();

        telemetry.addData("int1", int1);
        telemetry.addData("int2", int2);
        telemetry.addData("int3", int3);
        telemetry.addData("out1", out1);
        telemetry.addData("out2", out2);
        telemetry.addData("out3", out3);
        telemetry.addData("INT" , INT);
        telemetry.addData("OUT" , OUT);
        telemetry.update();


    }
    private void UPD() {
        boolean dpl = gamepad1.dpad_left;
        boolean dpr = gamepad1.dpad_right;
        boolean dpu = gamepad1.dpad_up;
        boolean dpd = gamepad1.dpad_down;

        if (dpl && !ldpl) {
            INT += 0.001;
        }

        if (dpr && !ldpr) {

            INT-= 0.001;
        }

        ldpl = dpl;
        ldpr = dpr;
        if (dpu && !ldpu) {
            INT += 0.01;
        }

        if (dpd && !ldpd) {
            INT -= 0.01;
        }

        ldpu = dpu;
        ldpd = dpd;
    }
    private void IndexPose() {
        if(gamepad1.left_bumper) {
            Indexer.setPosition(INT);
        }
        if(gamepad1.right_bumper) {
            Indexer.setPosition(OUT);
        }
    }

}