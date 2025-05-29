package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public class SingleProcessorPipelineManager<T extends VisionProcessor, I> implements VisionProcessorPipelineManager<I> {
    private final VisionProcessorType<T> type;
    private final T processor;
    private final I identifier;

    public SingleProcessorPipelineManager(VisionProcessorType<T> type, T processor, I identifier) {
        this.type = type;
        this.processor = processor;
        this.identifier = identifier;
    }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType) {
        return visionProcessorType == type ? Optional.of((T) processor) : Optional.empty();
    }

    @Override
    public <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType) {
        return visionProcessorType == type ? Optional.of(identifier) : Optional.empty();
    }

    @Override
    public <T extends VisionProcessor> boolean hasProcessor(VisionProcessorType<T> processorType) {
        return type == processorType;
    }
}
