package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.VisionProcessorPipelineManager;
import frc.robot.subsystems.vision.io.api.VisionSettings;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

public interface ProcessorPipelineRegistry<I> {
    <T extends VisionProcessor> void addPipelineID(I identifier, VisionProcessorType<T> visionProcessorType);

    <T extends VisionProcessor> void setPipelineSettings(I identifier, VisionProcessorType<T> visionProcessorType, VisionSettings<T> visionSettings);

    <T extends VisionProcessor> void registerProcessor(VisionProcessorType<T> visionProcessorType, T processor);

    <T extends VisionProcessor> boolean supports(VisionProcessorType<T> visionProcessorType);

    boolean hasPipelineID(I identifier);

    int pipelineCount();

    VisionProcessorPipelineManager<I> toPipelineManager();
}
