package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.limelight.util.LimelightHelpers;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class LimelightBuilder extends AbstractIndexedVisionHandleBuilder<LimelightBuilder> {
    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    public LimelightBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    @Override
    protected void switchPipeline(int index) {
        LimelightHelpers.setPipelineIndex(cameraID.cameraName, index);
    }

    @Override
    protected LimelightBuilder getThis() {
        return this;
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(VisionAprilTagSettingsConfigurator.VisionAprilTagSettings aprilTagMode) {
        return new LimelightVisionAprilTag(
                cameraID.cameraName,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new LimelightVisionNN(cameraID.cameraName, classNames);
    }
}
