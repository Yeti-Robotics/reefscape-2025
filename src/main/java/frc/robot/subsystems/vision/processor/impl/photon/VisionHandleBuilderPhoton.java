package frc.robot.subsystems.vision.processor.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.processor.AbstractVisionHandleBuilder;
import frc.robot.subsystems.vision.processor.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.processor.VisionAprilTagSettings;
import frc.robot.subsystems.vision.processor.VisionNNProcessor;
import org.photonvision.PhotonCamera;

import java.util.function.Supplier;

/**
 * Specialized builder for PhotonVision cameras.
 * This builder creates processors specifically for PhotonVision cameras.
 */
public class VisionHandleBuilderPhoton extends AbstractVisionHandleBuilder {
    private final PhotonCamera photonCamera;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     * @param visionSubsystem
     */
    private VisionHandleBuilderPhoton(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, PhotonCamera photonCamera, VisionHandlePhoton handle, VisionSubsystem visionSubsystem) {
        super(cameraID, drivetrainRotation, robotToCameraTransform, handle, visionSubsystem);
        this.photonCamera = photonCamera;
    }

    public static VisionHandleBuilderPhoton createBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, VisionSubsystem visionSubsystem) {
        PhotonCamera photonCamera = new PhotonCamera(cameraID.cameraName);
        VisionHandlePhoton handle = new VisionHandlePhoton(cameraID, photonCamera);

        return new VisionHandleBuilderPhoton(
                cameraID, drivetrainRotation, robotToCameraTransform,
                photonCamera, handle, visionSubsystem
        );
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(VisionAprilTagSettings.VisionAprilTagMode aprilTagMode) {
        return new VisionAprilTagPhoton(
                photonCamera,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new VisionNNPhoton(photonCamera, classNames);
    }
}
