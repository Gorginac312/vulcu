package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class LimelightSubOld {

    private final Limelight3A limelight;
    private LLResult result;
    public LLResult getresult() {
        return result;}

    public LimelightSubOld(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }



    public double getHeadingCorrection(boolean control) {
        if (control) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                return 0;
            }

            return result.getTx() * 0.04;
        }
        else return 0;
    }
    public double getYawCorrection(boolean control) {
        if (control) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) return 0;

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            if (fiducials == null || fiducials.isEmpty()) return 0;

            double yaw = fiducials.get(0)
                    .getTargetPoseCameraSpace()
                    .getOrientation()
                    .getYaw();

            return -yaw * 0.03;
        }
        else return 0;
    }
}


