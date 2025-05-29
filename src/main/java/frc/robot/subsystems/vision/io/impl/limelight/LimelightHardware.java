package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.io.api.VisionCameraHardware;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;

public class LimelightHardware implements VisionCameraHardware<Integer> {
    private static final String HEART_BEAT = "hb";
    private static final double HEART_BEAT_TIME = 0.5;

    private double lastHeartbeatValue = 0;
    private double lastHeartbeatTime = 0;

    private final String limelightName;

    public LimelightHardware(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public boolean isConnected() {
        double currentHeartBeatValue = LimelightHelpers.getLimelightNTDouble(limelightName, HEART_BEAT);
        double now = Timer.getFPGATimestamp();

        if (currentHeartBeatValue == 0) {
            return false;
        }

        if (currentHeartBeatValue != lastHeartbeatValue) {
            lastHeartbeatTime = now;
            lastHeartbeatValue = currentHeartBeatValue;
        }

        return (now - lastHeartbeatTime) < HEART_BEAT_TIME;
    }

    @Override
    public Integer getPipelineID() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    @Override
    public void setPipelineID(Integer pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }
}
