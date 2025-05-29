package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Specialized builder for PhotonVision cameras.
 * This builder creates processors specifically for PhotonVision cameras.
 */
public class PhotonVisionBuilder extends AbstractIndexedVisionHandleBuilder<PhotonHardware, PhotonVisionBuilder> {
    private final PhotonCamera photonCamera;
    private Consumer<SimCameraProperties> simCameraSettings;

    /**
     * Creates a new vision handle builder for a predefined camera ID.
     *
     * @param cameraID               The camera ID
     * @param drivetrainRotation     Supplier for the drivetrain rotation
     * @param robotToCameraTransform The transform from robot to camera
     */
    private PhotonVisionBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform, PhotonCamera photonCamera) {
        super(cameraID, drivetrainRotation, robotToCameraTransform);
        this.photonCamera = photonCamera;
    }

    public static PhotonVisionBuilder createBuilder(VisionCameraID cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        PhotonCamera photonCamera = new PhotonCamera(cameraID.cameraName);

        return new PhotonVisionBuilder(
                cameraID, drivetrainRotation, robotToCameraTransform,
                photonCamera
        );
    }

    public PhotonVisionBuilder withSimCameraSettings(Consumer<SimCameraProperties> cameraSettings) {
        this.simCameraSettings = cameraSettings;
        return this;
    }

    @Override
    protected PhotonVisionBuilder getThis() {
        return this;
    }

    @Override
    protected PhotonHardware createCameraHardware() {
        return new PhotonHardware(photonCamera);
    }


    @Override
    protected VisionAprilTag3DProcessor createAprilTagProcessor(VisionAprilTagSettings aprilTagMode) {
        return new PhotonVisionAprilTag3D(
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
