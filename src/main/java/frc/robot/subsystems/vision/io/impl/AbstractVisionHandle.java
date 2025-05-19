package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessorManager;

public abstract class AbstractVisionHandle implements VisionHandle {
    protected final VisionCameraID cameraID;
    private final VisionProcessorManager delegate;

    public AbstractVisionHandle(VisionCameraID cameraID, VisionProcessorManager delegate) {
        this.delegate = delegate;
        this.cameraID = cameraID;
    }

    @Override
    public VisionCameraID identifier() {
        return cameraID;
    }

    @Override
    public VisionProcessorManager vision() {
        return delegate;
    }
}
