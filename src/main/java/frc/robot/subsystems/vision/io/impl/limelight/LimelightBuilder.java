package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.impl.AbstractDefaultVisionHandleBuilder;

import java.util.function.Supplier;

/**
 * Specialized builder for Limelight cameras.
 * This builder creates processors specifically for Limelight cameras.
 */
public class LimelightBuilder extends AbstractDefaultVisionHandleBuilder<LimelightHardware, LimelightBuilder> {
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
    protected LimelightBuilder getThis() {
        return this;
    }

    @Override
    protected LimelightHardware createCameraHardware() {
        return new LimelightHardware(cameraID.cameraName);
    }

    @Override
    protected VisionAprilTag3DProcessor createAprilTagProcessor(VisionAprilTagSettings visionAprilTagSettings) {
        return new LimelightVisionAprilTag3D(
                cameraID.cameraName,
                robotToCameraTransform,
                drivetrainRotation,
                visionAprilTagSettings);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new LimelightVisionNN(cameraID.cameraName);
    }
}
