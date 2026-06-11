package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerSub {
    private DriveSub drive;
    private final Servo Indexer;
    private final RevColorSensorV3 Beam;
    private final DcMotor OuttakeContinu;
    private final DcMotorEx MO1;
    //manual
    private boolean lastintake = false;
    private boolean lastouttake = false;
    private boolean lastAutoOn = false;
    private int o = 0;
    private int i = 0;
    //auto
    private ElapsedTime IndexTime = new ElapsedTime();
    private int IndexState = 0;
    private ElapsedTime fullTimer = new ElapsedTime();
    //senzor
    private boolean objectDetected = false;
    private boolean sensorEnabled = false;
    public int balls = 0;
    private boolean full = false;
    private boolean LastToggle = false;
    public boolean getfull() {
        return full;
    }
    public int getballs() {
        return balls;
    }
    public boolean getSensorEnabled() {
        return sensorEnabled;
    }
    public double GetIndexState() { return IndexState;}
    public IndexerSub(HardwareMap hardwareMap) {
        Indexer = hardwareMap.get(Servo.class , "Indexer");
        Beam = hardwareMap.get(RevColorSensorV3.class , "Beam");
        OuttakeContinu = hardwareMap.get(DcMotor.class , "OuttakeContinu");
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        Indexer.setPosition(0.0);
        drive = new DriveSub(hardwareMap);
    }
    public void FullReset() {
        if(full && fullTimer.seconds() > 1.0) {
            full = false;
            balls = 0;
        }
    }
    public void SensorToggle(boolean Toggle){
        if(full) {
            sensorEnabled = false;//se opreste automat dupa 3 bile//
        }
        if(Toggle && !LastToggle && !full) {
            sensorEnabled = !sensorEnabled;//daca nu sunt 3 bile si este apasat joystick left sensorEnabled devine true//
        }
        LastToggle = Toggle;
        if(sensorEnabled) {
            Sensor();
        }
    }
    public void Sensor() {
        double d = Beam.getDistance(DistanceUnit.CM);
        boolean isClose = (d < 2);
        if(isClose && !objectDetected) {
            i++;
            if(i > 3) i = 1;
            updateIndexer();
            balls++;
            if(balls >= 3){
                sensorEnabled = false;
                full = true;
            }
        }
        objectDetected = isClose;
    }
    public void updateIndexer() {

        if(i == 1)      Indexer.setPosition(ValuesSub.intakepos1);
        else if(i == 2) Indexer.setPosition(ValuesSub.intakepos2);
        else            Indexer.setPosition(ValuesSub.intakepos3);
    }

    public void IndexManual(boolean intake,boolean outtake) {
        if (intake && !lastintake) {
            o = 0;

            i++;
            if(i > 3) i = 1;

            if(i == 1)      Indexer.setPosition(ValuesSub.intakepos1);
            else if(i == 2) Indexer.setPosition(ValuesSub.intakepos2);
            else            Indexer.setPosition(ValuesSub.intakepos3);
        }

        if(outtake && !lastouttake) {
            i = 0;

            o++;
            if(o > 3) o = 1;

            if(o == 1)      Indexer.setPosition(ValuesSub.outtakepos1);
            else if(o == 2) Indexer.setPosition(ValuesSub.outtakepos2);
            else            Indexer.setPosition(ValuesSub.outtakepos3);
        }

        lastintake = intake;
        lastouttake = outtake;
    }
    public void autoindex(boolean AutoOn) {
        if(AutoOn && !lastAutoOn && IndexState == 0) {
            IndexState = 1;
            Indexer.setPosition(ValuesSub.outtakepos1);
            IndexTime.reset();
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
            IndexState = 0;
            IndexTime.reset();
        }

        lastAutoOn = AutoOn;

    }
    public void AutonomySensorToggle(){
        if(full) {
            sensorEnabled = false;//se opreste automat dupa 3 bile//
        }
        if(!full) {
            sensorEnabled = !sensorEnabled;//daca nu sunt 3 bile si este apasat joystick left sensorEnabled devine true//
        }
        if(sensorEnabled) {
            Sensor();
        }
    }

    public void autonomyintake() {
        drive.drive(-0.2,0,0,1.0);
        sensorEnabled = true;
        AutonomySensorToggle();
        if(!sensorEnabled){drive.drive(0,0,0,1.0);}

    }
    public void autonomyouttake() {
        if (IndexState == 0) {
            IndexState = 1;
            Indexer.setPosition(ValuesSub.outtakepos1);
            IndexTime.reset();
        }
        if (IndexState != 0) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            MO1.setVelocity(ValuesSub.targetOUT);
        }
        if (IndexState == 1 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos2);
            IndexTime.reset();
            IndexState = 2;
        }
        if (IndexState == 2 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos3);
            IndexTime.reset();
            IndexState = 3;
        }
        if (IndexState == 3 && IndexTime.seconds() > 0.5) {
            IndexState = 0;
            IndexTime.reset();
            MO1.setVelocity(0.0);
            OuttakeContinu.setPower(0.0);
        }

    }

}
