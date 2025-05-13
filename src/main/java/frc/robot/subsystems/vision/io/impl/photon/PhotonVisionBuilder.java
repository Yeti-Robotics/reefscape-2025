package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionCameraID;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.photon.sim.AprilTagCamSim;
import frc.robot.subsystems.vision.io.impl.photon.sim.AprilTagCamSimBuilder;
import frc.robot.subsystems.vision.io.impl.photon.sim.AprilTagSimulator;
import org.photonvision.PhotonCamera;

import java.util.function.Supplier;

/**
 * Specialized builder for PhotonVision cameras.
 * This builder creates processors specifically for PhotonVision cameras.
 */
public class PhotonVisionBuilder extends AbstractIndexedVisionHandleBuilder {
    private final PhotonCamera photonCamera;

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
        PhotonCamera photonCamera;

        if (Robot.isSimulation()) {
            AprilTagCamSim camSim = AprilTagCamSimBuilder.newCamera()
                    .withCameraName(cameraID.cameraName)
                    .withTransform(robotToCameraTransform)
                    .build();

            AprilTagSimulator.getInstance().addCamera(camSim);
            photonCamera = camSim.getCamera();
        } else {
            photonCamera = new PhotonCamera(cameraID.cameraName);
        }

        return new PhotonVisionBuilder(
                cameraID, drivetrainRotation, robotToCameraTransform,
                photonCamera
        );
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
        return new PhotonVisionHandle(cameraID, photonCamera, visionProcessors.toArray(VisionProcessor[]::new));
    }
}
