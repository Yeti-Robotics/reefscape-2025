package frc.robot.subsystems.vision.io.impl.limelight;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandle;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;

/**
 * Specialized vision handle for Limelight cameras.
 * This class provides Limelight-specific functionality for switching pipelines.
 */
public class LimelightHandle extends AbstractIndexedVisionHandle {

    /**
     * Creates a Limelight vision handle for a predefined camera ID.
     * 
     * @param camera The camera ID
     */
    public LimelightHandle(VisionCameraID camera, VisionProcessor[] visionProcessors) {
        super(camera, visionProcessors);
    }

    @Override
    protected void switchPipeline(int pipelineIdentifier) {
        LimelightHelpers.setPipelineIndex(cameraID.cameraName, pipelineIdentifier);
    }
}