package frc.robot.subsystems.vision.io.api.processor;

import java.util.Optional;

public interface VisionProcessorManager {
    /**
     * Sets the current processor type.
     * This should also switch the camera pipeline if applicable.
     *
     * @param processorType The processor type to set as current
     * @return if setting the current processor was successful (true) or not (false)
     */
    boolean setCurrentProcessor(VisionProcessorType<? extends VisionProcessor> processorType);

    /**
     * Gets the active vision processor's type
     * @return {@link VisionProcessorType}
     */
    VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType();

    /**
     * Gets a specific type of processor from this handle.
     *
     * @param <T> The type of processor to get
     * @param processorType The processor type to get
     * @return The processor of the specified type, or null if none exists
     */
    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> processorType);

    /**
     * Checks if this handle has a processor of the specified type.
     *
     * @param processorType The processor type to check for
     * @return True if the processor exists, false otherwise
     */
    boolean hasProcessor(VisionProcessorType<? extends VisionProcessor> processorType);
}
