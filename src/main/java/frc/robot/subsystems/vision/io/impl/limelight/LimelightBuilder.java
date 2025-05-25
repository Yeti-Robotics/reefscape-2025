package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.api.processor.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;
import frc.robot.subsystems.vision.io.impl.pipeline.PipelineManager;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class LimelightBuilder extends AbstractIndexedVisionHandleBuilder<LimelightHandle, LimelightBuilder> {
    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public LimelightBuilder(VisionCameraID<LimelightHandle, LimelightBuilder> cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    @Override
    protected LimelightBuilder getThis() {
        return this;
    }

    @Override
    protected PipelineManager<Integer> createPipelineManager() {
        return new PipelineManager<>() {
            @Override
            public Integer getPipeline() {
                return (int) LimelightHelpers.getCurrentPipelineIndex(cameraID.cameraName);
            }

            @Override
            public void setPipeline(Integer pipeline) {
                LimelightHelpers.setPipelineIndex(cameraID.cameraName, pipeline);
            }
        };
    }

    @Override
    protected VisionAprilTag3DProcessor createAprilTagProcessor(VisionAprilTagSettings aprilTagMode) {
        return new LimelightVisionAprilTag3D(
                cameraID.cameraName,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new LimelightVisionNN(cameraID.cameraName, classNames);
    }

    @Override
    protected LimelightHandle buildWithManager(VisionProcessorManager processorManager) {
        return new LimelightHandle(cameraID, processorManager, cameraID.cameraName);
    }
}
