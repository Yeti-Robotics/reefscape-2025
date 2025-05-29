package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public interface ProcessorPipelineRegistry<I> {
    <T extends VisionProcessor> void addPipeline(VisionProcessorType<T> visionProcessorType, I identifier, T visionProcessor);

    <T extends VisionProcessor> Optional<I> getPipelineID(VisionProcessorType<T> visionProcessorType);

    boolean hasPipelineID(I identifier);

    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType);

    int pipelineCount();

    VisionProcessorPipelineManager<I> toPipelineManager();
}
