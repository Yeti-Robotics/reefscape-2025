package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.*;

public class ProcessorPipelineRegistryImpl<I> implements ProcessorPipelineRegistry<I>, VisionProcessorPipelineManager<I> {
    private record PipelineProcessorPair<T extends VisionProcessor, I>(T processor, I identifier){}

    private final Map<VisionProcessorType<? extends VisionProcessor>, PipelineProcessorPair<? extends VisionProcessor, I>> processors = new HashMap<>();
    private Set<I> existingPipelineIDs = new HashSet<>();

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

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType) {
        PipelineProcessorPair<T, I> pipelineProcessorPair = (PipelineProcessorPair<T, I>) processors.get(visionProcessorType);

        if (pipelineProcessorPair == null) {
            return Optional.empty();
        }

        return Optional.of(pipelineProcessorPair.processor);
    }

    @Override
    public <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType) {
        return getPair(visionProcessorType)
                .map(r -> r.identifier);
    }

    @Override
    public <T extends VisionProcessor> boolean hasProcessor(VisionProcessorType<T> processorType) {
        return processors.containsKey(processorType);
    }

    @Override
    public int pipelineCount() {
        return processors.size();
    }

    @Override
    public boolean hasPipelineID(I identifier) {
        return existingPipelineIDs.contains(identifier);
    }

    @SuppressWarnings("unchecked")
    private <T extends VisionProcessor> SingleProcessorPipelineManager<T, I> singleProcessorManager() {
        Map.Entry<VisionProcessorType<? extends VisionProcessor>, PipelineProcessorPair<? extends VisionProcessor, I>> pipelineProcessorPairEntry =
                processors.entrySet().iterator().next();

        VisionProcessorType<T> processorType = (VisionProcessorType<T>) pipelineProcessorPairEntry.getKey();
        T processor = (T) pipelineProcessorPairEntry.getValue().processor;

        return new SingleProcessorPipelineManager<>(processorType, processor, pipelineProcessorPairEntry.getValue().identifier);
    }

    @Override
    public VisionProcessorPipelineManager<I> toPipelineManager() {
        if (pipelineCount() == 1) {
            return singleProcessorManager();
        }

        existingPipelineIDs = null;

        return this;
    }
}
