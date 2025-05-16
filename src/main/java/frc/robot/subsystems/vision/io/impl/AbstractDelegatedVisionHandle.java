package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;

import java.util.Optional;

// for use with custom vision handle code
public class AbstractDelegatedVisionHandle implements VisionHandle {
    private final VisionHandle delegate;

    public AbstractDelegatedVisionHandle(VisionHandle delegate) {
        this.delegate = delegate;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return delegate.setCurrentProcessor(processorType);
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        return delegate.activeVisionProcessorType();
    }

    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
        return delegate.getProcessor(processorType);
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return delegate.hasProcessor(processorType);
    }

    @Override
    public VisionCameraID getCameraID() {
        return delegate.getCameraID();
    }
}
