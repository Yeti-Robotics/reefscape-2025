package frc.robot.subsystems.vision.io.impl.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionCameraID;

import java.util.function.Supplier;

public class LimelightCameraID extends VisionCameraID<LimelightHandle, LimelightBuilder> {
    public LimelightCameraID(String cameraName) {
        super(cameraName, VisionType.LIMELIGHT_MEGATAG_2);
    }

    @Override
    protected LimelightBuilder createCameraBuilder(VisionCameraID<LimelightHandle, LimelightBuilder> cameraID, Supplier<Rotation2d> drivetrainRotation, Transform3d robotToCameraTransform) {
        return new LimelightBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }
}
