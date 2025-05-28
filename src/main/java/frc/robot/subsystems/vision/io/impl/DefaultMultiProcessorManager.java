package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineProcessor;

import java.util.Map;
import java.util.Optional;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public class DefaultMultiProcessorManager<I> implements VisionProcessorManager {
    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;
    protected VisionProcessorType<? extends VisionProcessor> queuedProcessor;

    protected final Map<VisionProcessorType<? extends VisionProcessor>, PipelineProcessor<? extends VisionProcessor, I>> processorMap;
    protected final PipelineManager<I> pipelineManager;

    public DefaultMultiProcessorManager(Map<VisionProcessorType<? extends VisionProcessor>, PipelineProcessor<? extends VisionProcessor, I>> processorMap,
                                        PipelineManager<I> pipelineManager,
                                        VisionProcessorType<? extends VisionProcessor> initialProcessor) {
        this.currentProcessor = initialProcessor;
        this.processorMap = processorMap;
        this.pipelineManager = pipelineManager;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        PipelineProcessor<? extends VisionProcessor, I> visionProcessorInfo = processorMap.get(processorType);

        if (visionProcessorInfo == null || visionProcessorInfo.getPipelineIdentifier() == null) return false;

        pipelineManager.setPipeline(visionProcessorInfo.getPipelineIdentifier());
        queuedProcessor = processorType;
        return true;
    }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
        PipelineProcessor<T, I> visionProcessor = (PipelineProcessor<T, I>) processorMap.get(processorType);

        if (visionProcessor != null) {
            return Optional.of(visionProcessor.getProcessor());
        }

        return Optional.empty();
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return processorMap.containsKey(processorType);
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        if (queuedProcessor != null) {
            PipelineProcessor<? extends VisionProcessor, I> visionProcessorData = processorMap.get(queuedProcessor);

            if (visionProcessorData != null && visionProcessorData.getPipelineIdentifier().equals(pipelineManager.getPipeline())) {
                currentProcessor = queuedProcessor;
                queuedProcessor = null;
            }
        }

        return currentProcessor;
    }
}