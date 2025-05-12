package frc.robot.subsystems.vision.processor.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.processor.AbstractVisionHandleBuilder;
import frc.robot.subsystems.vision.processor.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.processor.VisionAprilTagSettings;
import frc.robot.subsystems.vision.processor.VisionNNProcessor;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class VisionHandleBuilderLimelight extends AbstractVisionHandleBuilder {


    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     * @param visionSubsystem
     */
    public VisionHandleBuilderLimelight(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, VisionSubsystem visionSubsystem) {
        super(cameraID, drivetrainRotation, robotToCameraTransform, new VisionHandleLimelight(cameraID), visionSubsystem);
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(VisionAprilTagSettings.VisionAprilTagMode aprilTagMode) {
        return new VisionAprilTagLimelight(
                cameraName,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new VisionNNLimelight(cameraName, classNames);
    }
}
