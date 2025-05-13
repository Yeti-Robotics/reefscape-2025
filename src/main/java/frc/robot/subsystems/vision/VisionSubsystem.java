package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.data.*;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagFeature;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightBuilder;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionBuilder;

import java.util.*;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final Map<VisionCameraID, VisionHandle> visionHandles = new EnumMap<>(VisionCameraID.class);

    // apriltag data/configs
    protected final Supplier<Rotation2d> drivetrainRotation;
    // since these are the two most commonly used features, initialize by default
    private final NavigableMap<Double, VisionData<VisionRobotPose>> visionPoses = new ConcurrentSkipListMap<>();
    private final List<VisionData<VisionAprilTag>> bestDetections = new ArrayList<>();
    private NavigableMap<Double, VisionData<VisionAprilTag>> aprilTagAllDetectionsMap = null;

    // neural net detections
    private NavigableMap<Double, VisionData<VisionNNDetection>> nnDetectionsMap = null;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        drivetrainRotation = () -> drivetrain.getPigeon2().getRotation2d();
    }

    public <T extends VisionProcessor> Optional<T> getProcessor(VisionCameraID cameraID,
                                                                VisionProcessorType<T> processorType) {
        return visionHandles.get(cameraID).getProcessor(processorType);
    }

    private void checkCameraID(VisionCameraID visionCameraID, VisionCameraID.VisionType requiredType) {
        if (visionCameraID.visionType != requiredType) {
            throw new IllegalArgumentException("Camera ID must be of type " + requiredType.name());
        }
    }

    public LimelightBuilder createLimelightCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        checkCameraID(cameraID, VisionCameraID.VisionType.LIMELIGHT_MEGATAG_2);
        return new LimelightBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public PhotonVisionBuilder createPhotonVisionCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        checkCameraID(cameraID, VisionCameraID.VisionType.PHOTONVISION);
        return PhotonVisionBuilder.createBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public List<VisionData<VisionAprilTag>> getBestDetections() {
        bestDetections.clear();

        for (VisionHandle handle : visionHandles.values()) {
            if (handle.activeVisionProcessorType() == VisionProcessorType.APRILTAG) {
                handle.getProcessor(VisionProcessorType.APRILTAG)
                        .flatMap(VisionAprilTagProcessor::getBestAprilTagObservation)
                        .ifPresent(visionAprilTag -> bestDetections.add(new VisionData<>(handle.getCameraID(), visionAprilTag)));
            }
        }

        return bestDetections;
    }

    public Optional<VisionData<VisionRobotPose>> pollVisionPoseUpdate() {
        Map.Entry<Double, VisionData<VisionRobotPose>> entry = visionPoses.pollFirstEntry();
        return entry == null ? Optional.empty() : Optional.of(entry.getValue());
    }

    public void addVisionHandle(VisionHandle... handles) {
        for (VisionHandle handle : handles) {
            visionHandles.put(handle.getCameraID(), handle);

            if (aprilTagAllDetectionsMap == null) {
                boolean enabledAllDetections = handle.getProcessor(VisionProcessorType.APRILTAG)
                        .map(VisionAprilTagProcessor::getSettings)
                        .filter(s -> s.hasEnabled(VisionAprilTagFeature.ALL_DETECTIONS))
                        .isPresent();

                if (enabledAllDetections) {
                    aprilTagAllDetectionsMap = new ConcurrentSkipListMap<>();
                }
            }

            if (nnDetectionsMap == null && handle.hasProcessor(VisionProcessorType.NN)) {
                nnDetectionsMap = new ConcurrentSkipListMap<>();
            }
        }
    }

    private static <T extends VisionTimestampedResult> void populateMap(NavigableMap<Double, VisionData<T>> map, List<T> entries, VisionCameraID cameraID) {
        for (T entry : entries) {
            map.put(entry.timestamp(), new VisionData<>(cameraID, entry));
        }
    }

    public static <T extends VisionProcessor> Optional<T> getProcessor(VisionHandle handle, VisionProcessorType<T> processorType) {
        Optional<T> processor = handle.getProcessor(processorType);
        processor.ifPresent(VisionProcessor::visionPeriodic);
        return processor;
    }

    @Override
    public void periodic() {
        for (VisionHandle handle : visionHandles.values()) {
            if (handle.activeVisionProcessorType() == VisionProcessorType.APRILTAG) {
                Optional<VisionAprilTagProcessor> aprilTagProcessor = getProcessor(handle, VisionProcessorType.APRILTAG);

                if (aprilTagProcessor.isPresent()) {
                    VisionAprilTagProcessor visionAprilTagProcessor = aprilTagProcessor.get();
                    VisionAprilTagSettings settings = visionAprilTagProcessor.getSettings();

                    if (settings.hasEnabled(VisionAprilTagFeature.LOCALIZATION)) {
                        populateMap(visionPoses, visionAprilTagProcessor.getRobotPoseObservation(), handle.getCameraID());
                    }

                    if (settings.hasEnabled(VisionAprilTagFeature.ALL_DETECTIONS) && aprilTagAllDetectionsMap != null) {
                        populateMap(aprilTagAllDetectionsMap, visionAprilTagProcessor.getLatestAprilTagObservations(), handle.getCameraID());
                    }
                }
            } else if (handle.activeVisionProcessorType() == VisionProcessorType.NN) {
                Optional<VisionNNProcessor> nnProcessor = getProcessor(handle, VisionProcessorType.NN);

                if (nnProcessor.isPresent()) {
                    VisionNNProcessor visionNNProcessor = nnProcessor.get();

                    if (nnDetectionsMap != null) {
                        populateMap(nnDetectionsMap, visionNNProcessor.getLatestNNDetections(), handle.getCameraID());
                    }
                }
            }
        }
    }
}
