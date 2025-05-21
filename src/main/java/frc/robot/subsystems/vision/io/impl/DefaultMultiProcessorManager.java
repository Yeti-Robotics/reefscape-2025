package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorManager;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;

import java.util.Map;
import java.util.Optional;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public class DefaultMultiProcessorManager<I> implements VisionProcessorManager {
    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;
    protected VisionProcessorType<? extends VisionProcessor> queuedProcessor;

    protected final Map<VisionProcessorType<? extends VisionProcessor>, AbstractBaseVisionHandleBuilder.VisionProcessorData<? extends VisionProcessor, I>> processorMap;
    protected final PipelineManager<I> pipelineManager;

    public DefaultMultiProcessorManager(Map<VisionProcessorType<? extends VisionProcessor>, AbstractBaseVisionHandleBuilder.VisionProcessorData<? extends VisionProcessor, I>> processorMap,
                                        PipelineManager<I> pipelineManager,
                                        VisionProcessorType<? extends VisionProcessor> initialProcessor) {
        this.currentProcessor = initialProcessor;
        this.processorMap = processorMap;
        this.pipelineManager = pipelineManager;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        AbstractBaseVisionHandleBuilder.VisionProcessorData<? extends VisionProcessor, I> visionProcessorInfo = processorMap.get(processorType);

        if (visionProcessorInfo == null || visionProcessorInfo.getPipelineID() == null) return false;

        pipelineManager.setPipeline(visionProcessorInfo.getPipelineID());
        queuedProcessor = processorType;
        return true;
    }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
        AbstractBaseVisionHandleBuilder.VisionProcessorData<T, I> visionProcessor = (AbstractBaseVisionHandleBuilder.VisionProcessorData<T, I>)
                processorMap.get(processorType);

        if (visionProcessor != null) {
            return Optional.of(visionProcessor.processor);
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
            AbstractBaseVisionHandleBuilder.VisionProcessorData<? extends VisionProcessor, I> visionProcessorData = processorMap.get(queuedProcessor);

            if (visionProcessorData != null && visionProcessorData.getPipelineID().equals(pipelineManager.getPipeline())) {
                currentProcessor = queuedProcessor;
                queuedProcessor = null;
            }
        }

        return currentProcessor;
    }
}