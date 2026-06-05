package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSub {

    private DcMotor LF, RF, LB, RB;

    public DriveSub(HardwareMap hardwareMap) {
        LB = hardwareMap.get(DcMotor.class , "LB");
        RB = hardwareMap.get(DcMotor.class , "RB");
        LF = hardwareMap.get(DcMotor.class , "LF");
        RF = hardwareMap.get(DcMotor.class , "RF");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double axial,double lateral,double yaw,double slowMode) {

        //constante de putere pentru roti//
        double lf = (axial + lateral + yaw) * slowMode;
        double rf = (axial - lateral - yaw) * slowMode;
        double lb = (axial - lateral + yaw) * slowMode;
        double rb = (axial + lateral - yaw) * slowMode;
        //formula(poti ignora//
        double max = Math.max(Math.abs(lf),
                Math.max(Math.abs(rf),
                        Math.max(Math.abs(lb), Math.abs(rb))));

        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }
        //da putere la roti in functie de constante//
        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }
}