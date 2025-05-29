package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.*;

public class PipelineProcessorRegistryImpl<I> implements PipelineProcessorRegistry<I> {
    private record PipelineProcessorPair<T extends VisionProcessor, I>(T processor, I identifier){}

    private final Map<VisionProcessorType<? extends VisionProcessor>, PipelineProcessorPair<? extends VisionProcessor, I>> processors = new HashMap<>();
    private final Set<I> existingPipelineIDs = new HashSet<>();

    @Override
    public <T extends VisionProcessor> void addPipeline(VisionProcessorType<T> visionProcessorType, I identifier, T visionProcessor) {
        if (existingPipelineIDs.contains(identifier)) {
            throw new IllegalArgumentException("Duplicate pipeline identifier: " + identifier);
        }

        processors.put(visionProcessorType, new PipelineProcessorPair<>(visionProcessor, identifier));
        existingPipelineIDs.add(identifier);
    }

    @SuppressWarnings("unchecked")
    private <T extends VisionProcessor> Optional<PipelineProcessorPair<T, I>> getPair(VisionProcessorType<T> visionProcessorType) {
        return Optional.ofNullable((PipelineProcessorPair<T, I>) processors.get(visionProcessorType));
    }

    @Override
    public <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType) {
        return getPair(visionProcessorType)
                .map(r -> r.identifier);
    }

    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType) {
        return getPair(visionProcessorType)
                .map(r -> r.processor);
    }

    @Override
    public int pipelineCount() {
        return processors.size();
    }

    @Override
    public <T extends VisionProcessor> boolean hasProcessor(VisionProcessorType<T> visionProcessorType) {
        return processors.containsKey(visionProcessorType);
    }
}
