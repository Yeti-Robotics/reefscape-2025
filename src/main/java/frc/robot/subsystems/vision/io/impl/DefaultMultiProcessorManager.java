package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineProcessorRegistry;

import java.util.Optional;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public class DefaultMultiProcessorManager<I> implements VisionProcessorManager {
    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;
    protected VisionProcessorType<? extends VisionProcessor> queuedProcessor;

    protected final PipelineProcessorRegistry<I> pipelineProcessorRegistry;
    protected final PipelineManager<I> pipelineSwitcher;

    public DefaultMultiProcessorManager(PipelineProcessorRegistry<I> pipelineProcessorRegistry,
                                        PipelineManager<I> pipelineManager,
                                        VisionProcessorType<? extends VisionProcessor> initialProcessor) {
        this.currentProcessor = initialProcessor;
        this.pipelineProcessorRegistry = pipelineProcessorRegistry;
        this.pipelineSwitcher = pipelineManager;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        Optional<I> identifierOpt = pipelineProcessorRegistry.getPipelineID(processorType);

        if (identifierOpt.isPresent()) {
            I identifier = identifierOpt.get();
            pipelineSwitcher.setPipeline(identifier);
            queuedProcessor = processorType;
            return true;
        }

        return true;
    }

    @Override
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
        return pipelineProcessorRegistry.getProcessor(processorType);
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return pipelineProcessorRegistry.hasProcessor(processorType);
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        if (queuedProcessor != null) {
            Optional<I> identifierOpt = pipelineProcessorRegistry.getPipelineID(queuedProcessor);

            if (identifierOpt.isPresent()) {
                I identifier = identifierOpt.get();

                if (pipelineSwitcher.getPipeline().equals(identifier)) {
                    currentProcessor = queuedProcessor;
                    queuedProcessor = null;
                }
            }
        }

        return currentProcessor;
    }
}