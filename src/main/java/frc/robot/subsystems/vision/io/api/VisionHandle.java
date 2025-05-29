package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorType;

import java.util.Optional;

/**
 * Interface for vision handles.
 * Provides methods for managing vision processors and switching pipelines.
 */
public interface VisionHandle {
    VisionCameraID identifier();

    boolean isConnected();

    <T extends VisionProcessor> Optional<T> getProcessor(VisionProcessorType<T> visionProcessorType);

    VisionProcessorType<? extends VisionProcessor> activeVisionProcessorType();

    <T extends VisionProcessor> boolean switchToProcessor(VisionProcessorType<T> visionProcessorType);
}

