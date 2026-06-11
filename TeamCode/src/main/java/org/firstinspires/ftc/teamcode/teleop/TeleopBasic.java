package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ValuesSub;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp (name = "TeleopBasic", group = "Test")
public class TeleopBasic extends OpMode {

    //MOTOARE//
    DcMotor LB;//left back wheel//
    DcMotor RB;//right back wheel//
    DcMotor LF;//left front wheel//
    DcMotor RF;//right front wheel//
    DcMotorEx IntakeMotor;//motor intake//
    DcMotor OuttakeContinu;//outtake elastice//
    DcMotorEx MO1;//outtake flywheel//
    DcMotor Elevate;//elevate, nefolosit//
    Servo Indexer;//indexer//
    RevColorSensorV3 Beam;//senzor distanta/culoare//
    Limelight3A limelight;
    LLResult result;

    //INDEXER//
    boolean lastY = false;
    boolean lastB = false;
    int i = 0;//intake//
    int o = 0;//outtake//

    //ENCODER//
    double tick_rpm = 28;//constanta pt motor//
    double rpm_fly = 2500;//target rpm//
    double rpm_far = 3600;//variabila pt target rpm//
    double rpm_close = 3000;//variabila pt target rpm//
    double current_fly = 0;//pt telemetrie(masoara rpm)//
    double current_intake = 0;//la fel dar pt intake//
    double intake_target = 2000;//intake target rpm//
    double velocity = 0;//valoare rpm pt fly//

    //CONTROALE RPM//
    boolean ldpu = false;//last dpad up//
    boolean ldpd = false;//last dpad down//
    boolean ldpl = false;//last dpad left//
    boolean ldpr = false;//last dpad right//
    boolean ljoyl = false;//last left joystick(apasat)//
    double slowMode = 0.4;//mers sasiu mai incet//

    //VALORI SENZOR//
    boolean objectDetected = false;//daca vede bila//
    boolean sensorEnabled = false;//opreste senzor//
    int balls = 0;//nr bile in indexer//
    boolean full = false;//devine adevarat cand indexerul e full(3 bile)//
    ElapsedTime fullTimer = new ElapsedTime();//reseteaza full si balls dupa un interval de timp(fara interval de timp nu merge)//
    ElapsedTime IndexTime = new ElapsedTime();//interval de timp intre indexari la auto//

    //AUTO INDEX//
    boolean cross = false;//x de pe controller ps//
    boolean lcross = false;//last cross//
    int IndexState = 0;//6 state-uri pentru intervale de timp intre indexare si outtake//
    boolean lrsb = false;

    //LIMELIGHT
    double yaw = 0;
    double txk =0.08;
    double yawk = 0.15;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    double correcttx = 0;
    double correctyaw = 0;

    @Override
    public void init() {

        //DECLARATII//

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
        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        limelight.pipelineSwitch(0);//0 = apriltag , 1 = game piece , 2 = custom
        limelight.start();


        //schimba directia motoarelor//
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        MO1.setDirection(DcMotor.Direction.REVERSE);

        //nu lasa sa mearga din inertie//
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MO1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pozitie indexer basic//
        Indexer.setPosition(0.0);

        //motoare cu encoder(folosind rpm in loc de puterea bateriei)//
        MO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //tuning pentru motoare cu encoder//

        //P (Proportional)
        //Too low → sluggish response, takes long to reach target RPM
        //Too high → oscillation/hunting around target RPM
        //Most common cause of rapid RPM bouncing

        //I (Integral)
        //Fixes steady-state error
        //Too high → slow large oscillations or “windup”
        //Usually less of a fast jitter and more of a drifting overshoot

        //D (Derivative)
        //Dampens oscillation
        //Too high → noisy/unresponsive system
        //Often added after tuning P

        //F (Feedforward)
        //Base power needed to maintain speed
        //Wrong F usually causes consistent underspeed/overspeed, not oscillation by itself

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

        result = limelight.getLatestResult();
        Chassis();
        Intake();
        Outtake();
        Outtake2();
        Indexer();
        SensorToggle();
        autoindex();
        limelight();
        double ticksPerSecond = MO1.getVelocity();
        double ticksPerSecondIntake = IntakeMotor.getVelocity();
        velocity = (rpm_fly * tick_rpm)/60;
        current_fly = (ticksPerSecond * 60) / tick_rpm;
        current_intake = (ticksPerSecondIntake * 60)/tick_rpm;

        //TELEMETRY(AFISAT PE CONSOLA//

        telemetry.addData("Target RPM", rpm_fly);
        telemetry.addData("Current RPM", current_fly);
        telemetry.addData("intake rpm", current_intake);
        telemetry.addData("intake target" , intake_target);
        telemetry.addData("sensorEnabled", sensorEnabled);
        telemetry.addData("full", full);
        telemetry.addData("balls", balls);
        telemetry.addData("right stick button", gamepad1.right_stick_button);
        telemetry.addData("apriltag", result.isValid());
        telemetry.addData("tx", tx);
        telemetry.addData("yaw", yaw);
        telemetry.addData("correcttx", correcttx);
        telemetry.addData("correctyaw", correctyaw);
        telemetry.update();

        //RESET PENTRU AUTO INDEX//

        if(full && fullTimer.seconds() > 1.0) {
            full = false;
            balls = 0;
        }


        return ticksPerSecond;
    }
    public void limelight() {

        boolean rsb = gamepad1.right_stick_button;

        if (result == null) {
            return;
        }

        if (!result.isValid() || !rsb) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return;
        }

        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

        correcttx = tx * ValuesSub.txconstant;

        if (fiducials.get(0).getTargetPoseCameraSpace() != null) {
            yaw = fiducials.get(0)
                    .getTargetPoseCameraSpace()
                    .getOrientation()
                    .getYaw();
        }

        correctyaw = -yaw * ValuesSub.yawconstant;
        if (Math.abs(correcttx) < 0.05) correcttx = 0;
        if (Math.abs(correctyaw) < 0.05) correctyaw = 0;

        LF.setPower(correcttx - correctyaw);
        LB.setPower(-correcttx - correctyaw);
        RF.setPower(-correcttx + correctyaw);
        RB.setPower(correcttx + correctyaw);
    }


    public void autoindex() {
        cross = gamepad1.cross;
        if(cross && !lcross && IndexState == 0) {
            Indexer.setPosition(ValuesSub.outtakepos1);
            IndexTime.reset();
            IndexState = 1;
        }
        //asteapta 0.5 sec si executa//
        if(IndexState == 1 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            IndexTime.reset();
            IndexState = 2;
            //la fel la toate dar se schimba IndexState ca sa se schimbe if-ul executat//
        }
        if(IndexState == 2 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(0.0);
            Indexer.setPosition(ValuesSub.outtakepos2);
            IndexTime.reset();
            IndexState = 3;
        }
        if(IndexState == 3 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            IndexTime.reset();
            IndexState = 4;
        }
        if(IndexState == 4 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            Indexer.setPosition(ValuesSub.outtakepos3);
            IndexTime.reset();
            IndexState = 5;
        }
        if(IndexState == 5 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            IndexTime.reset();
            IndexState = 6;
        }
        if(IndexState == 6 && IndexTime.seconds() > 0.5) {
            OuttakeContinu.setPower(0.0);
            IndexState = 0;
        }

        lcross = cross;
    }  public void SensorToggle() {
        boolean joyl = gamepad1.left_stick_button;
        if(full) {
            sensorEnabled = false;//se opreste automat dupa 3 bile//
        }
        if(joyl && !ljoyl && !full) {
            sensorEnabled = !sensorEnabled;//daca nu sunt 3 bile si este apasat joystick left sensorEnabled devine true//
        }
        ljoyl = joyl;
        if(sensorEnabled) {
            Sensor();
        }
    }
    public void updateIndexer() {
        //valori intake indexer pentru senzor//

        if(i == 1)      Indexer.setPosition(ValuesSub.intakepos1);
        else if(i == 2) Indexer.setPosition(ValuesSub.intakepos2);
        else            Indexer.setPosition(ValuesSub.intakepos3);
    }
    public void Sensor() {
        double d = Beam.getDistance(DistanceUnit.CM);//distanta masurata intre senzor si obiect//
        boolean isClose = (d < 2);//isClose este adevarat daca sunt mai putin de 2cm intre senzor si obiect//
        if(isClose && !objectDetected) {//daca distanta este mai mica de 2cm se detecteaza obiect(objectDetected = true)//
            i++;
            if(i > 3) i = 1;
            updateIndexer();//schimba pozitia de indexer//
            balls++;
            if(balls >= 3){//oprire automata la 3 bile luate//
                sensorEnabled = false;
                full = true;
            }
        }
        objectDetected = isClose;//daca isclose = true obiect este detectat//
    }
    public void Indexer() {
        //o = intake, i = outtake (contori);
        boolean currentY = gamepad1.y;
        boolean currentB = gamepad1.b;
        if (currentY && !lastY) {
            o = 0;

            i++;
            if(i > 3) i = 1;

            if(i == 1)      Indexer.setPosition(ValuesSub.intakepos1);
            else if(i == 2) Indexer.setPosition(ValuesSub.intakepos2);
            else            Indexer.setPosition(ValuesSub.intakepos3);
        }

        if(currentB && !lastB) {
            i = 0;

            o++;
            if(o > 3) o = 1;

            if(o == 1)      Indexer.setPosition(ValuesSub.outtakepos1);
            else if(o == 2) Indexer.setPosition(ValuesSub.outtakepos2);
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
            rpm_fly = rpm_far;//schimba target rpm la far//
        }
        ldpu = dpu;

        if (dpd && !ldpd) {
            rpm_fly = rpm_close;//schimba target rpm la close//
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
        double IntakeVelocity = (intake_target * tick_rpm)/60;
        if(gamepad1.left_trigger > 0.5) {
            IntakeMotor.setVelocity(ValuesSub.targetINT);
        }
        else {
            IntakeMotor.setVelocity(0.0);
        }
        if (dpl && !ldpl) {
            intake_target -= 100;//scade 100 rpm la intake//
        }
        ldpl = dpl;

        if (dpr && !ldpr) {
            intake_target += 100;//adauga 100 rpm la intake//
        }
        ldpr = dpr;
    }
    public void Chassis() {
        //sasiu(doar formule nu invata)//

        //MISCARE//

        double axial = -gamepad1.left_stick_y;//fata spate//
        double lateral = gamepad1.left_stick_x;//stanga dreapta//
        double yaw = gamepad1.right_stick_x;//rotatie//
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
