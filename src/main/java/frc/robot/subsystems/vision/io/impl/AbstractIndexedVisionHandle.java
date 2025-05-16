package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;

import java.util.Optional;
import java.util.function.IntConsumer;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public class AbstractIndexedVisionHandle implements VisionHandle {
    protected final VisionCameraID cameraID;
    protected final VisionProcessor[] visionProcessors;
    protected final IntConsumer pipelineSwitcher;

    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;
    private int pipelineIndex = 0;


    /**
     * Creates a vision handle for a predefined camera ID.
     *
     * @param cameraID The camera ID
     */
    public AbstractIndexedVisionHandle(VisionCameraID cameraID, VisionProcessor[] visionProcessors, IntConsumer pipelineSwitcher, VisionProcessorType<? extends VisionProcessor> initialProcessor) {
        this.cameraID = cameraID;
        this.visionProcessors = visionProcessors;
        this.pipelineSwitcher = pipelineSwitcher;
        this.currentProcessor = initialProcessor;
    }

    private <T extends VisionProcessor> int getProcessorIndex(VisionProcessorType<T> processorType) {
        for (int i = 0; i < visionProcessors.length; i++) {
            if (processorType.clazz.isInstance(visionProcessors[i])) {
                return i;
            }
        }

        return -1;
    }

    @Override
    public boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        if (visionProcessors.length == 1) return false;

        int pipelineIndex = getProcessorIndex(processorType);

        if (pipelineIndex == -1) return false;

        this.pipelineIndex = pipelineIndex;
        pipelineSwitcher.accept(pipelineIndex);
        currentProcessor = processorType;

        return true;
    }

    @SuppressWarnings("unchecked")
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
        if (processorType == currentProcessor && visionProcessors.length == 1) return Optional.of((T) visionProcessors[0]);
        if (processorType == currentProcessor && visionProcessors.length > 1) return Optional.of((T) visionProcessors[pipelineIndex]);
        int index = getProcessorIndex(processorType);

        return index == -1 ? Optional.empty() : Optional.of((T) visionProcessors[index]);
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return (processorType == currentProcessor && visionProcessors.length == 1) || getProcessorIndex(processorType) != -1;
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        return currentProcessor;
    }

    @Override
    public VisionCameraID getCameraID() {
        return cameraID;
    }
}