package frc.robot.subsystems.vision.io.impl.photon;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;

import java.util.function.Supplier;

public class PhotonVisionCameraID extends VisionCameraID<PhotonVisionHandle, PhotonVisionBuilder> {
    public PhotonVisionCameraID(String cameraName) {
        super(cameraName, VisionType.PHOTONVISION);
    }

    @Override
    protected PhotonVisionBuilder createCameraBuilder(VisionCameraID<PhotonVisionHandle, PhotonVisionBuilder> cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        return PhotonVisionBuilder.createBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }
}
