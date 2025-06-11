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

    Optional<VisionProcessorType<? extends VisionProcessor>> getVisionProcessorType(I pipelineID);

    <T extends VisionProcessor> VisionSettings<T> getSettings(I pipelineID);
}
