package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public class DefaultSingleProcessorManager<T extends VisionProcessor> implements VisionProcessorManager {
    private final VisionProcessorType<T> processorType;
    private final VisionProcessor visionProcessor;

    public DefaultSingleProcessorManager(VisionProcessorType<T> processorType, VisionProcessor visionProcessor) {
        if (!processorType.clazz.isInstance(visionProcessor)) {
            throw new IllegalArgumentException("The processor type does not match the processor instance!");
        }

        this.processorType = processorType;
        this.visionProcessor = visionProcessor;
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
}
