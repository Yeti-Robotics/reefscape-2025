package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.AprilTagVisionSettings;
import org.photonvision.PhotonCamera;

import java.util.function.Supplier;

/**
 * Specialized builder for PhotonVision cameras.
 * This builder creates processors specifically for PhotonVision cameras.
 */
public class PhotonVisionBuilder extends AbstractVisionHandleBuilder<Integer> {
    private final PhotonCamera photonCamera;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     * @param visionSubsystem
     */
    private PhotonVisionBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, PhotonCamera photonCamera, PhotonVisionHandle handle, VisionSubsystem visionSubsystem) {
        super(cameraID, drivetrainRotation, robotToCameraTransform, handle, visionSubsystem);
        this.photonCamera = photonCamera;
    }

    public static PhotonVisionBuilder createBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, VisionSubsystem visionSubsystem) {
        PhotonCamera photonCamera = new PhotonCamera(cameraID.cameraName);
        PhotonVisionHandle handle = new PhotonVisionHandle(cameraID, photonCamera);

        return new PhotonVisionBuilder(
                cameraID, drivetrainRotation, robotToCameraTransform,
                photonCamera, handle, visionSubsystem
        );
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(AprilTagVisionSettings.VisionAprilTagMode aprilTagMode) {
        return new PhotonVisionAprilTag(
                photonCamera,
                robotToCameraTransform,
                drivetrainRotation,
                aprilTagMode);
    }

    @Override
    protected VisionNNProcessor createNNProcessor(String[] classNames) {
        return new PhotonVisionNN(photonCamera, classNames);
    }
}
