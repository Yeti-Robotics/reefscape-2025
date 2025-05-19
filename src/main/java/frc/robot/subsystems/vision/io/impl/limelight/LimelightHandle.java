package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionProcessorManager;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandle;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;

public class LimelightHandle extends AbstractVisionHandle {
    private static final String HEART_BEAT = "hb";
    private static final double HEART_BEAT_TIME = 0.5;

    private double lastHeartbeatValue = 0;
    private double lastHeartbeatTime = 0;

    private final String cameraName;

    public LimelightHandle(VisionCameraID cameraID, VisionProcessorManager delegate, String cameraName) {
        super(cameraID, delegate);
        this.cameraName = cameraName;
    }

    @Override
    public boolean isConnected() {
        double currentHeartBeatValue = LimelightHelpers.getLimelightNTDouble(cameraName, HEART_BEAT);
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
}
