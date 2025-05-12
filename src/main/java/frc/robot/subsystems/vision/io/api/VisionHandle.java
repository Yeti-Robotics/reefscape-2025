package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.pipeline.PipelineIdentifier;
import frc.robot.subsystems.vision.io.pipeline.VisionProcessorType;

/**
 * Interface for vision handles.
 * Provides methods for managing vision processors and switching pipelines.
 */
public interface VisionHandle<P> {

    /**
     * Adds a processor to this handle.
     *
     * @param <T> The type of processor
     * @param processorType The processor type
     * @param identifier The pipeline identifier associated with the processor
     * @param processor The processor instance
     */
    <T extends VisionProcessor> void addProcessor(VisionProcessorType<T> processorType, PipelineIdentifier<P> identifier, T processor);

    /**
     * Sets the current processor type.
     * This should also switch the camera pipeline if applicable.
     *
     * @param processorType The processor type to set as current
     */
    void setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType);

    /**
     * Gets the active vision processor.
     *
     * @return The currently active vision processor, or null if none is set
     */
    VisionProcessor activeVisionProcessor();

    /**
     * Gets a specific type of processor from this handle.
     *
     * @param <T> The type of processor to get
     * @param processorType The processor type to get
     * @return The processor of the specified type, or null if none exists
     */
    <T extends VisionProcessor> T getProcessor(VisionProcessorType<T> processorType);

    /**
     * Checks if this handle has a processor of the specified type.
     *
     * @param processorType The processor type to check for
     * @return True if the processor exists, false otherwise
     */
    boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType);

    VisionCameraID getCameraID();
}

