package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.VisionSettings;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public class SingleProcessorPipelineManager<V extends VisionProcessor, I> implements VisionProcessorPipelineManager<I> {
    private final VisionProcessorType<V> type;
    private final V processor;
    private final I identifier;

    public SingleProcessorPipelineManager(VisionProcessorType<V> type, V processor, I identifier) {
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
    public Optional<VisionProcessorType<? extends VisionProcessor>> getVisionProcessorType(I pipelineID) {
        return identifier.equals(pipelineID) ? Optional.of(type) : Optional.empty();
    }

    @Override
    public <T extends VisionProcessor> VisionSettings<T> getSettings(I pipelineID) {
        return null;
    }
}
