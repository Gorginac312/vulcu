package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "limelight", group = "Test")
public class limelight extends OpMode {

    Limelight3A limelight;
    DcMotor LB;
    DcMotor RB;
    DcMotor LF;
    DcMotor RF;



    public void init() {
        LB = hardwareMap.get(DcMotor.class , "LB");
        RB = hardwareMap.get(DcMotor.class , "RB");
        LF = hardwareMap.get(DcMotor.class , "LF");
        RF = hardwareMap.get(DcMotor.class , "RF");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);//0 = apriltag , 1 = game piece , 2 = custom
        limelight.start();

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop(){

        //LLResult ofera toate datele pe care le vede limelight ul, deci daca:
        LLResult result = limelight.getLatestResult();

        //obiectul result contine:
        //daca exista un apriltag sau nu in frame(valid/nevalid)
        //id ul tag ului
        //x-cat de stanga sau dreapta se afla apriltag ul de centrul camerei
        //y-cat de sus sau jos se afla apriltagul de centrul camerei
        //a-cat % din camera este ocupata de apriltag
        //yaw,pitch/roll-cat de rotit este apriltagul fata de camera in diferite directii(masurat in grade, yaw=stanga dreapta , pitch=sus jos)
        //latency-cat de vechi este frameul analizat
        //pozitia robotului(?)

        //daca exista apriltag in frame ofera valorile fata de el
        if(result != null && result.isValid() && gamepad1.cross)
        {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            double turnPower = tx * 0.02;

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("area", ta);
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Cross", gamepad1.cross);
            telemetry.addData("Valid", result.isValid());
            telemetry.addData("tx", result.getTx());
            telemetry.addData("Cross", gamepad1.cross);
            telemetry.update();

            //da putere motoarelor in functie de x ca sa se alinieze cu apriltag ul la mijloc
            if (Math.abs(tx) < 1.0) {
                turnPower = 0;
            }
            LB.setPower(-turnPower);
            LF.setPower(turnPower);
            RB.setPower(-turnPower);
            RF.setPower(turnPower);


        }
    }
}

