package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.photon.sim.PhotonVisionAprilTagSimulator;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Specialized builder for PhotonVision cameras.
 * This builder creates processors specifically for PhotonVision cameras.
 */
public class PhotonVisionBuilder extends AbstractIndexedVisionHandleBuilder<PhotonVisionBuilder> {
    private final PhotonCamera photonCamera;
    private Consumer<SimCameraProperties> cameraSettings;

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
        this.cameraSettings = cameraSettings;
        return this;
    }

    @Override
    protected void switchPipeline(int index) {
        photonCamera.setPipelineIndex(index);
    }

    @Override
    protected PhotonVisionBuilder getThis() {
        return this;
    }

    @Override
    protected VisionAprilTagProcessor createAprilTagProcessor(VisionAprilTagSettingsConfigurator.VisionAprilTagSettings aprilTagMode) {
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

    @Override
    public VisionHandle build() {
        if (Robot.isSimulation()) {
            PhotonVisionAprilTagSimulator
                    .getInstance()
                    .ifPresent(sim -> sim.addCamera(cameraID, photonCamera, robotToCameraTransform, cameraSettings));
        }

        return super.build();
    }
}
