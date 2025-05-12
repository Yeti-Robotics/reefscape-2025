package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.AprilTagVisionSettings;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class LimelightBuilder extends AbstractVisionHandleBuilder {


    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     * @param visionSubsystem
     */
    public LimelightBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, VisionSubsystem visionSubsystem) {
        super(cameraID, drivetrainRotation, robotToCameraTransform, new LimelightHandle(cameraID), visionSubsystem);
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(AprilTagVisionSettings.VisionAprilTagMode aprilTagMode) {
        return new LimelightVisionAprilTag(
                cameraName,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new LimelightVisionNN(cameraName, classNames);
    }
}
