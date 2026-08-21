package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.SubSystems.ValuesSub.intakepos1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.DriveSub;
import org.firstinspires.ftc.teamcode.SubSystems.IndexerSub;


@TeleOp(name = "intake auto test", group = "Test")
public class intake_auto_test extends OpMode {

    boolean lastcross = false;
    IndexerSub indexer;
    Servo Indexer;
    DriveSub drive;

    @Override
    public void init() {
        drive = new DriveSub(hardwareMap);
        indexer = new IndexerSub(hardwareMap , drive);
        Indexer = hardwareMap.get(Servo.class , "Indexer");
        Indexer.setPosition(intakepos1);

    }
    @Override
    public void loop() {
    boolean cross = gamepad1.cross;
    if(cross && !lastcross) {
        indexer.autonomyintake();

    }
    }
}
