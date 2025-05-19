package frc.robot.subsystems.vision.io.api;

import frc.robot.subsystems.vision.VisionCameraID;

import java.util.Optional;

/**
 * Interface for vision handles.
 * Provides methods for managing vision processors and switching pipelines.
 */
public interface VisionHandle {
    VisionCameraID identifier();

    boolean isConnected();

    VisionProcessorManager vision();
}

