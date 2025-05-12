package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class LimelightBuilder extends AbstractIndexedVisionHandleBuilder {
    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     * @param visionSubsystem
     */
    public LimelightBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(AprilTagVisionSettings.AprilTagVisionMode aprilTagMode) {
        return new LimelightVisionAprilTag(
                cameraID.getCameraName(),
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new LimelightVisionNN(cameraID.getCameraName(), classNames);
    }

    @Override
    public VisionHandle build() {
        return new LimelightHandle(cameraID, visionProcessors.toArray(VisionProcessor[]::new));
    }
}
