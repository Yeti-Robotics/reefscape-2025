package frc.robot.subsystems.vision.io.impl;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.pipeline.PipelineIdentifier;
import frc.robot.subsystems.vision.io.pipeline.VisionProcessorType;

import java.util.HashMap;
import java.util.Map;

/**
 * Abstract base class for vision handles.
 * This class provides common functionality for all vision handles.
 */
public abstract class AbstractVisionHandle<P> {
    public final VisionCameraID cameraID;
    protected final Map<VisionProcessorType<? extends VisionProcessor>, VisionProcessor> ioMap = new HashMap<>();
    protected final Map<VisionProcessorType<? extends VisionProcessor>, PipelineIdentifier<P>> pipelineIdentifiers = new HashMap<>();
    protected VisionProcessorType<? extends VisionProcessor> currentProcessor;

    /**
     * Creates a vision handle for a predefined camera ID.
     * 
     * @param cameraID The camera ID
     */
    public AbstractVisionHandle(VisionCameraID cameraID) {
        this.cameraID = cameraID;
    }

    /**
     * Adds a processor to this handle.
     * 
     * @param <T> The type of processor
     * @param processorType The processor type
     * @param processor The processor instance
     */
    public <T extends VisionProcessor> void addProcessor(VisionProcessorType<T> processorType, PipelineIdentifier<P> identifier, T processor) {
        pipelineIdentifiers.put(processorType, identifier);
        ioMap.put(processorType, processor);
    }

    /**
     * Sets the current processor type.
     * This method also switches the camera pipeline if the processor has a pipeline identifier.
     * 
     * @param processorType The processor types to set as current
     */
    public void setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        if (hasProcessor(processorType)) {
            switchPipeline(pipelineIdentifiers.get(processorType));
            this.currentProcessor = processorType;
        }
    }

    /**
     * Switches the camera pipeline to the one associated with the specified processor.
     * 
     * @param processor The processor to switch to
     */
    protected abstract void switchPipeline(PipelineIdentifier<P> processor);

    /**
     * Gets the active vision processor.
     * 
     * @return The active vision processor, or null if no processor is set as current
     */
    public VisionProcessor activeVisionProcessor() {
        return ioMap.get(currentProcessor);
    }

    /**
     * Gets a specific type of processor from this handle.
     * 
     * @param <T> The type of processor to get
     * @param processorType The processor type to get
     * @return The processor of the specified type, or null if no processor of that type exists
     */
    @SuppressWarnings("unchecked")
    public <T extends VisionProcessor> T getProcessor(VisionProcessorType<T> processorType) {
        return (T) ioMap.get(processorType);
    }

    /**
     * Checks if this handle has a processor of the specified type.
     * 
     * @param processorType The processor type to check for
     * @return True if this handle has a processor of the specified type, false otherwise
     */
    public boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType) {
        return ioMap.containsKey(processorType);
    }
}