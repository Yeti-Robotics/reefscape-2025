package frc.robot.subsystems.vision.io.impl.pipeline;

import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

public interface PipelineProcessor<T extends VisionProcessor, I> {
    T getProcessor();
    I getPipelineIdentifier();
}
