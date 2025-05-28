package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public interface PipelineProcessorRegistry<I> {
    <T extends VisionProcessor> void addPipeline(VisionProcessorType<T> visionProcessorType, I identifier, T visionProcessor);

    <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType);

    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType);

    int pipelineCount();

    default <T extends VisionProcessor> boolean hasProcessor(VisionProcessorType<T> visionProcessorType) {
        return getProcessor(visionProcessorType).isPresent();
    }
}
