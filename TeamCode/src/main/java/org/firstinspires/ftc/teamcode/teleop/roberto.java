package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.SubSystems.ValuesSub;


@TeleOp(name = "sasiu_roberto", group = "Test")
public class roberto extends OpMode {
    DcMotor LB;
    DcMotor RB;
    DcMotor LF;
    DcMotor RF;
    DcMotor intake;
    CRServo IntServoL;
    CRServo IntServoR;


    int s = 1;

    boolean lcross = false;
    @Override
    public void init() {
        LB = hardwareMap.get(DcMotor.class , "LB");
        RB = hardwareMap.get(DcMotor.class , "RB");
        LF = hardwareMap.get(DcMotor.class , "LF");
        RF = hardwareMap.get(DcMotor.class , "RF");
        intake = hardwareMap.get(DcMotor.class, "intake");
        IntServoL = hardwareMap.get(CRServo.class, "IntServoL");
        IntServoR = hardwareMap.get(CRServo.class , "IntServoR");

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

    public void loop() {

        Chassis();
        Intake();
        Switch();
        Servo_Intake();

    }
    public void Servo_Intake() {
        if(gamepad1.right_trigger > 0.05) {
            IntServoL.setPower(1.0);
            IntServoR.setPower(-1.0);
        }
        else {
            IntServoL.setPower(0);
            IntServoR.setPower(0);
        }


    }
    public void Switch() {
        boolean cross = gamepad1.cross;
        if(cross && !lcross){
            s++;
        }
        if(s > 2)s = 1;
        if(s == 1) intake.setDirection(DcMotor.Direction.REVERSE);
        if(s == 2)intake.setDirection(DcMotor.Direction.FORWARD);
        lcross = cross;

    }
    public void Intake() {

        if (gamepad1.right_trigger > 0.05) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
    public void Chassis() {
        //sasiu(doar formule nu invata)//

        //MISCARE//

        double axial = -gamepad1.left_stick_y;//fata spate//
        double lateral = gamepad1.left_stick_x;//stanga dreapta//
        double yaw = gamepad1.right_stick_x;//rotatie//
        double slowMode;
        if(gamepad1.left_bumper) {
            slowMode = 0.4;//duce viteza la 0.4 din max la roti//
        }    else {
            slowMode = 1.0;//putere max la roti//
        }



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

