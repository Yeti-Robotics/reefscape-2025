package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.io.api.VisionAprilTagProcessor;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessor;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;
import frc.robot.subsystems.vision.data.VisionAprilTag;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.VisionTimestampedResult;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings.AprilTagVisionFeatures;
import frc.robot.subsystems.vision.io.api.AprilTagVisionSettings.AprilTagVisionMode;
import frc.robot.subsystems.vision.io.impl.AbstractIndexedVisionHandleBuilder;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightBuilder;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionBuilder;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final Map<VisionCameraID, VisionHandle> visionHandles = new EnumMap<>(VisionCameraID.class);

    // apriltag
    protected final Supplier<Rotation2d> drivetrainRotation;
    private final NavigableMap<Double, VisionRobotPose> visionPoses = new ConcurrentSkipListMap<>();
    private NavigableMap<Double, VisionAprilTag> allDetectionsMap = null;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        drivetrainRotation = () -> drivetrain.getPigeon2().getRotation2d();
    }

    /**
     * Gets a processor of the specified type for a predefined camera ID.
     * 
     * @param <T>           The type of processor to get
     * @param cameraID      The camera ID
     * @param processorType The processor type to get
     * @return An Optional containing the processor of the specified type, or empty
     *         if no processor exists
     */
    public <T extends VisionProcessor> Optional<T> getProcessor(VisionCameraID cameraID,
            VisionProcessorType<T> processorType) {
        return visionHandles.get(cameraID).getProcessor(processorType);
    }

    public LimelightBuilder createLimelightCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        return new LimelightBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public PhotonVisionBuilder createPhotonVisionCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        return PhotonVisionBuilder.createBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public void addVisionHandle(VisionHandle handle) {
        visionHandles.put(handle.getCameraID(), handle);

        if (allDetectionsMap == null) {
            Optional<VisionAprilTagProcessor> aprilTagProcessorOpt = handle.getProcessor(VisionProcessorType.APRILTAG);

            if (aprilTagProcessorOpt.isPresent()) {
                VisionAprilTagProcessor aprilTagProcessor = aprilTagProcessorOpt.get();

                if (aprilTagProcessor.getSettings().hasEnabled(AprilTagVisionFeatures.ALL_DETECTIONS)) {
                    allDetectionsMap = new ConcurrentSkipListMap<>();
                }
            }
        }
    }

    private static <T extends VisionTimestampedResult> void populateMap(NavigableMap<Double, T> map, List<T> entries) {
        for (T entry : entries) {
            map.put(entry.timestamp(), entry);
        }
    }

    @Override
    public void periodic() {
        // Call visionPeriodic on all processors
        for (VisionHandle handle : visionHandles.values()) {
            VisionProcessor processor = handle.activeVisionProcessor();

            if (processor != null) {
                processor.visionPeriodic();

                switch (processor) {
                    case VisionAprilTagProcessor aprilTagProcessor -> {
                        AprilTagVisionMode mode = aprilTagProcessor.getSettings();

                        if (mode.hasEnabled(AprilTagVisionFeatures.LOCALIZATION)) {
                            populateMap(visionPoses, aprilTagProcessor.getRobotPoseObservation());
                        }

                        if (mode.hasEnabled(AprilTagVisionFeatures.ALL_DETECTIONS) && allDetectionsMap != null) {
                            populateMap(allDetectionsMap, aprilTagProcessor.getLatestAprilTagObservations());
                        }
                    }
                    case VisionNNProcessor visionNNProcessor -> {

                    }
                }
            }
        }
    }
}
