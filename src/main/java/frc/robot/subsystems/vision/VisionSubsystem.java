package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.data.*;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTag3D;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagFeature;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.processor.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightBuilder;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionBuilder;

import java.util.*;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final Map<VisionCameraID, VisionHandle> visionHandles = new EnumMap<>(VisionCameraID.class);

    // apriltag data/configs
    private final Supplier<Rotation2d> drivetrainRotation;
    // since these are the two most commonly used features, initialize by default
    private final NavigableMap<Double, VisionData<VisionRobotPose>> visionPoses = new ConcurrentSkipListMap<>();
    private final List<VisionData<VisionAprilTag3D>> bestDetections = new ArrayList<>();
    private NavigableMap<Double, VisionData<VisionAprilTag3D>> aprilTagAllDetectionsMap = null;

    // neural net detections
    private NavigableMap<Double, VisionData<VisionNNDetection>> nnDetectionsMap = null;

    public VisionSubsystem(Supplier<Rotation2d> drivetrainRotation) {
        this.drivetrainRotation = drivetrainRotation;
    }

    public <T extends VisionProcessor> Optional<T> fetchProcessorForHandle(VisionCameraID cameraID,
                                                                           VisionProcessorType<T> processorType) {
        return visionHandles.get(cameraID).vision().getProcessor(processorType);
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

    public List<VisionData<VisionAprilTag3D>> getBestDetections() {
        bestDetections.clear();

        for (VisionHandle handle : visionHandles.values()) {
            VisionProcessorManager processorManager = handle.vision();
            if (processorManager.activeVisionProcessorType() == VisionProcessorType.APRILTAG_3D) {
                processorManager.getProcessor(VisionProcessorType.APRILTAG_3D)
                        .flatMap(VisionAprilTag3DProcessor::getBestAprilTagObservation)
                        .ifPresent(visionAprilTag -> bestDetections.add(new VisionData<>(handle.identifier(), visionAprilTag)));
            }
        }

        return bestDetections;
    }

    public Optional<VisionData<VisionRobotPose>> pollVisionPoseUpdate() {
        Map.Entry<Double, VisionData<VisionRobotPose>> entry = visionPoses.pollFirstEntry();
        return entry == null ? Optional.empty() : Optional.of(entry.getValue());
    }

    private static boolean hasEnabledAllDetections(VisionHandle handle) {
        return handle.vision().getProcessor(VisionProcessorType.APRILTAG_3D)
                .map(VisionAprilTag3DProcessor::getSettings)
                .filter(s -> s.hasEnabled(VisionAprilTagFeature.ALL_DETECTIONS))
                .isPresent();
    }

    public void addVisionHandle(VisionHandle... handles) {
        for (VisionHandle handle : handles) {
            visionHandles.put(handle.identifier(), handle);

            if (aprilTagAllDetectionsMap == null && hasEnabledAllDetections(handle)) {
                aprilTagAllDetectionsMap = new TreeMap<>();
            }

            if (nnDetectionsMap == null && handle.vision().hasProcessor(VisionProcessorType.NN)) {
                nnDetectionsMap = new TreeMap<>();
            }
        }
    }

    private static <T extends VisionTimestampedResult> void populateMap(NavigableMap<Double, VisionData<T>> map, List<T> entries, VisionCameraID cameraID) {
        for (T entry : entries) {
            map.put(entry.timestamp(), new VisionData<>(cameraID, entry));
        }
    }

    private static <T extends VisionProcessor> Optional<T> fetchProcessorForHandle(VisionHandle handle, VisionProcessorType<T> processorType) {
        Optional<T> processor = handle.vision().getProcessor(processorType);
        processor.ifPresent(VisionProcessor::visionPeriodic);
        return processor;
    }

    @Override
    public void periodic() {
        for (VisionHandle handle : visionHandles.values()) {
            VisionProcessorManager processorManager = handle.vision();

            if (processorManager.activeVisionProcessorType() == VisionProcessorType.APRILTAG_3D) {
                Optional<VisionAprilTag3DProcessor> aprilTagProcessor = fetchProcessorForHandle(handle, VisionProcessorType.APRILTAG_3D);

                if (aprilTagProcessor.isPresent()) {
                    VisionAprilTag3DProcessor visionAprilTag3DProcessor = aprilTagProcessor.get();
                    VisionAprilTagSettings settings = visionAprilTag3DProcessor.getSettings();

                    if (settings.hasEnabled(VisionAprilTagFeature.LOCALIZATION)) {
                        populateMap(visionPoses, visionAprilTag3DProcessor.getRobotPoseObservation(), handle.identifier());
                    }

                    if (settings.hasEnabled(VisionAprilTagFeature.ALL_DETECTIONS) && aprilTagAllDetectionsMap != null) {
                        populateMap(aprilTagAllDetectionsMap, visionAprilTag3DProcessor.getLatestAprilTagObservations(), handle.identifier());
                    }
                }
            } else if (processorManager.activeVisionProcessorType() == VisionProcessorType.NN) {
                Optional<VisionNNProcessor> nnProcessor = fetchProcessorForHandle(handle, VisionProcessorType.NN);

                if (nnProcessor.isPresent()) {
                    VisionNNProcessor visionNNProcessor = nnProcessor.get();

                    if (nnDetectionsMap != null) {
                        populateMap(nnDetectionsMap, visionNNProcessor.getLatestNNDetections(), handle.identifier());
                    }
                }
            }
        }
    }
}
