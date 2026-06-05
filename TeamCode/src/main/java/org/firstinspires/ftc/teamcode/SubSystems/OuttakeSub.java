package org.firstinspires.ftc.teamcode.SubSystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class OuttakeSub {

    private final DcMotorEx MO1;
    private final DcMotor OuttakeContinu;
    public OuttakeSub(HardwareMap hardwareMap) {
        OuttakeContinu = hardwareMap.get(DcMotor.class , "OuttakeContinu");
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        MO1.setDirection(DcMotor.Direction.REVERSE);
        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotorEx getMotor() {
        return MO1;
    }
    public void setPIDF() {
        MO1.setVelocityPIDFCoefficients(
                ValuesSub.OUTP,
                ValuesSub.OUTI,
                ValuesSub.OUTD,
                ValuesSub.OUTF
        );
    }

    public void update(boolean OuttakeOn,boolean close,boolean far) {
        if(close) ValuesSub.targetOUT = 3000;
        if(far) ValuesSub.targetOUT = 3600;
        double OuttakeRPM = (ValuesSub.targetOUT * ValuesSub.TicksPerRevOUT)/60;
        if(OuttakeOn) {
            MO1.setVelocity(OuttakeRPM);
        }
        else {
            MO1.setVelocity(0.0);
        }
    }

    public void update2(boolean OuttakeOn2){
        if(OuttakeOn2) {
            OuttakeContinu.setPower(0.5);
        }
        else {
            OuttakeContinu.setPower(0.0);
        }
    }


}