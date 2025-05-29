package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public interface VisionProcessorPipelineManager<I> {
    /**
     * Gets a specific type of processor from this handle.
     *
     * @param visionProcessorType The processor type to get
     * @return The processor of the specified type, or null if none exists
     */
    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType);


    <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType);

    /**
     * Checks if this handle has a processor of the specified type.
     *
     * @param processorType The processor type to check for
     * @return True if the processor exists, false otherwise
     */
    default <T extends VisionProcessor> boolean hasProcessor(VisionProcessorType<T> processorType) {
        return getProcessor(processorType).isPresent();
    }
}
