package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightBuilder;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightCameraID;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightHandle;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionBuilder;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionCameraID;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionHandle;

import java.util.function.Supplier;

public abstract class VisionCameraID<T extends VisionHandle, B extends VisionHandleBuilder> {
    public static final VisionCameraID<PhotonVisionHandle, PhotonVisionBuilder> RADIO_CAM = newPhotonVisionCamera("radio_cam");
    public static final VisionCameraID<PhotonVisionHandle, PhotonVisionBuilder> SCORE_CAM = newPhotonVisionCamera("score_cam");
    public static final VisionCameraID<LimelightHandle, LimelightBuilder> BELUGA_LIMELIGHT = newLimelightCamera("beluga_limelight");

    public enum VisionType {
        LIMELIGHT_MEGATAG_2,
        PHOTONVISION
    }

    private static VisionCameraID<PhotonVisionHandle, PhotonVisionBuilder> newPhotonVisionCamera(String cameraName) {
        return new PhotonVisionCameraID(cameraName);
    }

    private static VisionCameraID<LimelightHandle, LimelightBuilder> newLimelightCamera(String cameraName) {
        return new LimelightCameraID(cameraName);
    }

    public final String cameraName;
    public final VisionType visionType;

    protected VisionCameraID(String cameraName, VisionType visionType) {
        this.cameraName = cameraName;
        this.visionType = visionType;
    }

    protected abstract B createCameraBuilder(VisionCameraID<T, B> cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform);

    @Override
    public String toString() {
        return cameraName;
    }
}
