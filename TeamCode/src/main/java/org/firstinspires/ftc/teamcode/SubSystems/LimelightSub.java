package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class LimelightSub {

    private final Limelight3A limelight;
    private LLResult result;
    private Pose3D pose;
    public LLResult getresult() {
        return result;}
    double yaw;
    double tx;
    double ta;

    public LimelightSub(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

public double gettx() {
        return tx;
}
public double getyaw() {
        return yaw;
}

public double getTargetCorrection(boolean control) {
        if (control) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                return 0;
            }

            ta = result.getTa();
            if(ta == ValuesSub.targetarea) return 0;
            else return ta * ValuesSub.taconstant;

        }
        else return 0;
}
    public double getHeadingCorrection(boolean control) {
        if (control) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                return 0;
            }
            tx = result.getTx();
            return tx * ValuesSub.txconstant;
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

            fiducials = result.getFiducialResults();
            Pose3D pose = fiducials.get(0).getTargetPoseCameraSpace();

            yaw = pose.getOrientation().getYaw();

            return -yaw * ValuesSub.yawconstant;
        }
        else return 0;
    }
}

