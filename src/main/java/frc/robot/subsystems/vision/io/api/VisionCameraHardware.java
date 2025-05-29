package frc.robot.subsystems.vision.io.api;

public interface VisionCameraHardware<I> {
    boolean isConnected();
    I getPipelineID();
    void setPipelineID(I pipelineID);
}
