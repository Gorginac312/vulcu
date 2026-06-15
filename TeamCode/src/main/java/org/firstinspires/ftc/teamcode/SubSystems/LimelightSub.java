package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class LimelightSub {

    private final Limelight3A limelight;
    private Pose3D pose;
    private LLResult result;
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
public void update() {
        result = limelight.getLatestResult();
}

    public double getTargetCorrection(boolean control) {
        if (control) {
            if (result == null || !result.isValid()) {
                return 0;
            }
            ta = result.getTa();
            double error = ValuesSub.targetarea - ta;
            if (Math.abs(error) < ValuesSub.taTolerance) {
                return 0;
            }

            return error * ValuesSub.taconstant;
        } else {
            return 0;
        }
    }    public double getHeadingCorrection(boolean control) {
        if (control) {
            if (result == null || !result.isValid()) {
                return 0;
            }
            tx = result.getTx();
            double error = tx;
            if (Math.abs(error) < ValuesSub.txtolerance) {
                return 0;
            }

            return error * ValuesSub.txconstant;
        } else {
            return 0;
        }
    }
    public double getYawCorrection(boolean control) {
        if (control) {
            if (result == null || !result.isValid()) return 0;

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            if (fiducials == null || fiducials.isEmpty()) return 0;

            fiducials = result.getFiducialResults();
            pose = fiducials.get(0).getTargetPoseCameraSpace();

            yaw = pose.getOrientation().getYaw();
            double error = yaw;
            if (Math.abs(error) < ValuesSub.yawtolerance) {
                return 0;
            }

            return error * ValuesSub.yawconstant;
        } else {
            return 0;
        }
    }}

