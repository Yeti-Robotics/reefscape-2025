package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;

import java.util.Optional;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public abstract class AbstractIndexedVisionHandle implements VisionHandle {
    protected final VisionCameraID cameraID;
    protected final VisionProcessor[] visionProcessors;
    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;
    private int pipelineIndex = 0;


    /**
     * Creates a vision handle for a predefined camera ID.
     * @param cameraID The camera ID
     */
    public AbstractIndexedVisionHandle(VisionCameraID cameraID, VisionProcessor[] visionProcessors) {
        this.cameraID = cameraID;
        this.visionProcessors = visionProcessors;
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
    public void setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        int pipelineIndex = getProcessorIndex(processorType);

        if (pipelineIndex == -1) return;

        switchPipeline(pipelineIndex);
        this.pipelineIndex = pipelineIndex;
        currentProcessor = processorType;
    }

    @Override
    public VisionProcessor activeVisionProcessor() {
        return visionProcessors[pipelineIndex];
    }

    @SuppressWarnings("unchecked")
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType) {
      int index = getProcessorIndex(processorType);

      return index == -1 ? Optional.empty() : Optional.of((T) visionProcessors[index]);
    }

    @Override
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return getProcessorIndex(processorType) != -1;
    }

    @Override
    public VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType() {
        return currentProcessor;
    }

    @Override
    public VisionCameraID getCameraID() {
        return cameraID;
    }

    /**
     * Switches the camera pipeline to the one associated with the specified processor.
     * 
     * @param pipelineIndex The processor to switch to
     */
    protected abstract void switchPipeline(int pipelineIndex);
}