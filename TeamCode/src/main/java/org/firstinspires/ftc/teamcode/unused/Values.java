package org.firstinspires.ftc.teamcode.unused;




public class Values  {

    public Outtake outtake;

    //............PUSHSERVO.......//

    public static double PushStartPose = 0.32;//needs tunning
    public static double PushEndPose = 0.669;//needs tuning

    //...........LimeLight Servo.......///
    public static double TagServoStartPose = 0.38;

    // Close-aim presets (tune)
    public static double TagServoBlueClosePose = 0.59;  // <<< TUNE
    public static double TagServoRedClosePose  = 0.38;  // <<< TUNE (starting guess)



    //......INDEXER

    public static double TunPose=0.38;
    public static double IntakePose1 = 0.238;
    public static double IntakePose2 = 0.62;
    public static double IntakePose3 = 1;

    public static double OuttakePose1 = 0.06;
    public static double OuttakePose2 = 0.436;
    public static double OuttakePose3 = 0.821;

    //........OUTTAKE MOTORS......//

    public static double FLYWHEEL_TICKS_PER_REV = 28;


    public static double MAX_FLYWHEEL_RPM = 6000;

    // Shooting RPMs
    public static int CloseShootRPM = 2675;   // ~70% power equivalent
    public static int MidShootRPM   = 2800;   // ~53% power equivalent
    public static int FarShootRPM= 3630;

    //.......FlyWheelPID......///
    public static double F=10;
    public static double P=260;


    //.......Autonom Time Steps.....////

    public static double INDEXER_SETTLE_TIME = 220; // ms, configurable
    public static double Push_Open_Time = 170;
    public static double Push_Close_Time = 120;


    //......Intake.....///

    public static double IntakeRPM = 3300;           // default intake speed in RPM
    public static double INTAKE_TICKS_PER_REV = 28 ;


    // ================== AUTO HEADING TUNING (Limelight) ==================
    public static int TagPipeline = 5;

    public static double Heading_kP = 0.038;
    public static double Heading_kD = 0.002;

    public static double Heading_TxGoalDeg = -2;
    // ================= AUTO HEADING TX GOALS (per tag + range) =================
    public static double Heading_TxGoalBlueFarDeg   = -2.0;   // tag 20 far
    public static double Heading_TxGoalBlueCloseDeg = -5.7;   // tag 20 close
    public static double Heading_TxGoalRedFarDeg    = -4.0;   // tag 24 far
    public static double Heading_TxGoalRedCloseDeg  = -7.5;   // tag 24 close

    public static double Heading_DeadbandDeg = 0.8;

    public static double Heading_MaxRot = 0.45;

    public static boolean Heading_Invert = true;

    public static double Heading_MinRot = 0.1;

    // ================= BEAM SENSOR (AUTO INDEX) =================
    public static int BEAM_RAW_MIN = 200;
    public static int BEAM_RAW_MAX = 1000;
    public static int BEAM_PRESENT_LOOPS = 2;
    public static double BEAM_COOLDOWN_S = 0.200;
    public static double BEAM_REARM_S = 0.25;


    // ================= FLYWHEEL READY CHECK =================
    public static double FlywheelReadyTolRpm = 80;      // +/- RPM window
    public static double FlywheelReadyStableMs = 120;   // must be ready for this long






}
