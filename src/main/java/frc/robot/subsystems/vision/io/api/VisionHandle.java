package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;


public interface VisionHandle<I> {
    VisionCameraID identifier();

    boolean isConnected();

    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType);

    VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType();

    boolean switchToPipeline(I pipelineID);
}

