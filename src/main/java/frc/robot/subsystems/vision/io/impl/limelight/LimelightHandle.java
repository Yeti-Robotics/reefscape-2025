package frc.robot.subsystems.vision.io.impl.limelight;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandle;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;
import frc.robot.subsystems.vision.io.pipeline.PipelineIdentifier;

/**
 * Specialized vision handle for Limelight cameras.
 * This class provides Limelight-specific functionality for switching pipelines.
 */
public class LimelightHandle extends AbstractVisionHandle<Integer> {

    /**
     * Creates a Limelight vision handle for a predefined camera ID.
     * 
     * @param camera The camera ID
     */
    public LimelightHandle(VisionCameraID camera) {
        super(camera);
    }

    @Override
    protected void switchPipeline(PipelineIdentifier<Integer> pipelineIdentifier) {
        if (pipelineIdentifier != null) {
            LimelightHelpers.setPipelineIndex(cameraID.cameraName, pipelineIdentifier.getIdentifier());
        }
    }
}