package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;

import java.util.Optional;

public class AbstractSingleProcessorVisionHandle<T extends VisionProcessor> implements VisionHandle {
    private final VisionProcessorType<T> processorType;
    private final VisionProcessor visionProcessor;
    private final VisionCameraID cameraID;

    public AbstractSingleProcessorVisionHandle(VisionCameraID cameraID, VisionProcessor visionProcessor, VisionProcessorType<T> processorType) {
        if (!processorType.clazz.isInstance(visionProcessor)) {
            throw new IllegalArgumentException("The processor type does not match the processor instance!");
        }

        this.processorType = processorType;
        this.visionProcessor = visionProcessor;
        this.cameraID = cameraID;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return this.processorType == processorType;
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        return processorType;
    }

    @SuppressWarnings("unchecked")
    @Override
    public <P extends VisionProcessor> Optional<P> getProcessor(VisionProcessorType<P> processorType) {
        return processorType == this.processorType ? Optional.of((P) visionProcessor) : Optional.empty();
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return processorType == this.processorType;
    }

    @Override
    public VisionCameraID getCameraID() {
        return cameraID;
    }
}
