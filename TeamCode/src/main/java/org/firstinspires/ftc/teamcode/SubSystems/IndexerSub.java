package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.ValuesSub.intakepos1;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerSub {
    public DriveSub drive;
    private final Servo Indexer;
    private final Servo IntakeServo;
    private final RevColorSensorV3 Beam;
    private final DcMotor OuttakeContinu;
    private final DcMotorEx MO1;
    private final DcMotorEx IntakeMotor;
    //manual
    private boolean lastintake = false;
    private boolean lastouttake = false;
    private boolean lastAutoOn = false;
    public int o = 0;
    public int i = 0;
    //auto
    private ElapsedTime IndexTime = new ElapsedTime();
    private int IndexState = 0;
    private ElapsedTime fullTimer = new ElapsedTime();
    private ElapsedTime autointake = new ElapsedTime();
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
    double I = 0;

    private int detectionCount = 0;
    public IndexerSub(HardwareMap hardwareMap , DriveSub drivesub) {
        this.drive = drivesub;
        Indexer = hardwareMap.get(Servo.class , "Indexer");
        Beam = hardwareMap.get(RevColorSensorV3.class , "Beam");
        OuttakeContinu = hardwareMap.get(DcMotor.class , "OuttakeContinu");
        MO1 = hardwareMap.get(DcMotorEx.class , "MO1");
        IntakeMotor = hardwareMap.get(DcMotorEx.class , "IntakeMotor");
        IntakeServo = hardwareMap.get(Servo.class , "IntakeServo");
        Indexer.setPosition(0.0);
        IntakeServo.setPosition(0.65);

        MO1.setDirection(DcMotor.Direction.REVERSE);
        MO1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MO1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void IntakeServo(boolean toggle) {
        if(toggle) {IntakeServo.setPosition(0.5);}
        else {IntakeServo.setPosition(0.65);}
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
        boolean isClose = (d < 3.0 && d > 0.1); // Valid distance filter

        if (isClose) {
            detectionCount++;
        } else {
            detectionCount = 0; // Reset counter if object disappears
        }

        // Require 3 consecutive positive loops (~30ms) to register as a real object
        boolean confirmedClose = (detectionCount >= 3);

        if (confirmedClose && !objectDetected) {
            i++;
            if (i > 3) i = 1;
            updateIndexer();
            balls++;

            if (balls >= 3) {
                sensorEnabled = false;
                full = true;
            }
        }

        objectDetected = confirmedClose;
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
    public boolean autonomyintake() {
        double intrpm = (ValuesSub.targetINT * ValuesSub.TicksPerRevOUT) / 60.0;

        Sensor(); // Checks beam break / distance sensor

        if (full) {
            IntakeMotor.setPower(0.0);
            return true; // Finished intaking 3 balls
        }

        IntakeMotor.setPower(intrpm);
        return false; // Still running
    }
    public void startAutonomyIntake2() {
        i = 1;
        autointake.reset();
        updateIndexer();
    }
    public boolean autonomyintake2() {

        double intrpm =
                (ValuesSub.targetINT * ValuesSub.TicksPerRevOUT) / 60.0;

        IntakeMotor.setVelocity(intrpm);

        if (i == 1 && autointake.seconds() > 2) {
            i = 2;
            updateIndexer();
            autointake.reset();
        }

        if (i == 2 && autointake.seconds() > 2) {
            i = 3;
            updateIndexer();
            autointake.reset();
        }

        if (i == 3 && autointake.seconds() > 2) {
            IntakeMotor.setVelocity(0);
            i = 0;
            return true;
        }

        return false;
    }
    public void resetIndexState() {
        IndexState = 0;
        IndexTime.reset();
        full = false;
        balls = 0;
        i = 0;
        o = 0;
        sensorEnabled = true;
    }
    public void resetOuttakeState() {
        IndexState = 0;
        IndexTime.reset();
        o = 0;
        i = 0;
    }

    public boolean autonomyouttake() {
        sensorEnabled = false;
        // STATE 0: Start flywheel motor and reset timer
        if (IndexState == 0) {
            double outrpm = (ValuesSub.targetOUT * ValuesSub.TicksPerRevOUT) / 60.0;
            MO1.setVelocity(outrpm);
            IndexTime.reset(); // Start 1-second spin-up timer
            IndexState = 1;
            return false;
        }

        // STATE 1: Wait 1 second for flywheel to spin up, then push first ring/ball
        if (IndexState == 1 && IndexTime.seconds() > 1.0) {
            OuttakeContinu.setPower(ValuesSub.outtakepower);
            Indexer.setPosition(ValuesSub.outtakepos1);
            IndexTime.reset();
            IndexState = 2;
            return false;
        }

        // STATE 2: Push second ring/ball after 0.5s
        if (IndexState == 2 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos2);
            IndexTime.reset();
            IndexState = 3;
            return false;
        }

        // STATE 3: Push third ring/ball after 0.5s
        if (IndexState == 3 && IndexTime.seconds() > 0.5) {
            Indexer.setPosition(ValuesSub.outtakepos3);
            IndexTime.reset();
            IndexState = 4;
            return false;
        }

        // STATE 4: Clean up, turn off motors, and finish
        if (IndexState == 4 && IndexTime.seconds() > 0.5) {
            IndexState = 0; // Reset for next use

            MO1.setVelocity(0.0);
            OuttakeContinu.setPower(0.0);
            return true; // Finished outtake sequence!
        }

        return false;
    }
}
