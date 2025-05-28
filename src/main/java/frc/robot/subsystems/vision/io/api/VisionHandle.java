package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessorManager;

/**
 * Interface for vision handles.
 * Provides methods for managing vision processors and switching pipelines.
 */
public interface VisionHandle {
    VisionCameraID identifier();

    boolean isConnected();

    VisionProcessorManager vision();
}

