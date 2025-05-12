package frc.robot.subsystems.vision.processor.impl.limelight;

import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.processor.PipelineIdentifier;
import frc.robot.subsystems.vision.processor.VisionHandleAbstract;
import frc.robot.subsystems.vision.processor.impl.limelight.util.LimelightHelpers;

/**
 * Specialized vision handle for Limelight cameras.
 * This class provides Limelight-specific functionality for switching pipelines.
 */
public class VisionHandleLimelight extends VisionHandleAbstract<Integer> {

    /**
     * Creates a Limelight vision handle for a predefined camera ID.
     * 
     * @param camera The camera ID
     */
    public VisionHandleLimelight(VisionCameraID camera) {
        super(camera);
    }

    @Override
    protected void switchPipeline(PipelineIdentifier<Integer> pipelineIdentifier) {
        if (pipelineIdentifier != null) {
            LimelightHelpers.setPipelineIndex(cameraID.cameraName, pipelineIdentifier.getIdentifier());
        }
    }
}