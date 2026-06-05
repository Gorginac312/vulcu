package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeSub {
    private boolean lastincrease = false;
    private boolean lastdecrease = false;

    private final DcMotorEx IntakeMotor;
    public IntakeSub(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class , "IntakeMotor");
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotorEx getMotor() {
        return IntakeMotor;
    }
    public void setPIDF() {
        IntakeMotor.setVelocityPIDFCoefficients(
                ValuesSub.INTP,
                ValuesSub.INTI,
                ValuesSub.INTD,
                ValuesSub.INTF
        );
    }

    public void update(boolean IntakeOn,boolean increase,boolean decrease) {
        double IntakeRPM = (ValuesSub.targetINT * 28)/60;
        if(IntakeOn) {
            IntakeMotor.setVelocity(IntakeRPM);
        }
        else {
            IntakeMotor.setVelocity(0.0);
        }
        if(increase != lastincrease) ValuesSub.targetINT += 100;
        if(decrease != lastdecrease) ValuesSub.targetINT -= 100;
        lastincrease = increase;
        lastdecrease = decrease;
    }


}
