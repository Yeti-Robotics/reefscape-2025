package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.data.VisionData;
import frc.robot.subsystems.vision.data.VisionNNDetection;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.VisionTimestampedResult;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTag3D;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagTracker;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagFeature;
import frc.robot.subsystems.vision.io.api.VisionAprilTagSettingsConfigurator.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.VisionHandleBuilder;
import frc.robot.subsystems.vision.io.api.VisionProcessorManager;
import frc.robot.subsystems.vision.io.api.VisionProcessorType;
import frc.robot.subsystems.vision.io.api.processor.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.VisionProcessor;

import java.util.*;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final Map<VisionCameraID<? extends VisionHandle, ? extends VisionHandleBuilder>, VisionHandle> visionHandles = new HashMap<>();

    // apriltag data/configs
    private final Supplier<Rotation2d> drivetrainRotation;
    // since these are the two most commonly used features, initialize by default
    private final NavigableMap<Double, VisionData<VisionRobotPose>> visionPoses = new TreeMap<>();
    private NavigableMap<Double, VisionData<VisionAprilTag3D>> aprilTagDetectionsMap = new TreeMap<>();

    // neural net detections
    private NavigableMap<Double, VisionData<VisionNNDetection>> nnDetectionsMap = null;

    public VisionSubsystem(Supplier<Rotation2d> drivetrainRotation) {
        this.drivetrainRotation = drivetrainRotation;
    }

    public <T extends VisionHandle, B extends VisionHandleBuilder> B createCamera(VisionCameraID<T, B> cameraID, Transform3d robotToCameraTransform) {
        return cameraID.createCameraBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public Optional<VisionData<VisionRobotPose>> pollVisionPoseUpdate() {
        Map.Entry<Double, VisionData<VisionRobotPose>> entry = visionPoses.pollFirstEntry();
        return entry == null ? Optional.empty() : Optional.of(entry.getValue());
    }

    private static boolean hasEnabledAllDetections(VisionHandle handle) {
        return handle.vision().getProcessor(VisionProcessorType.APRILTAG_3D)
                .map(VisionAprilTag3DProcessor::getSettings)
                .filter(s -> s.hasEnabledAll(VisionAprilTagFeature.ALL_DETECTIONS))
                .isPresent();
    }

    @SuppressWarnings("unchecked")
    public void addVisionHandle(VisionHandle... handles) {
        for (VisionHandle handle : handles) {
            visionHandles.put(handle.identifier(), handle);

            if (aprilTagDetectionsMap == null && hasEnabledAllDetections(handle)) {
                aprilTagDetectionsMap = new TreeMap<>();
            }

            if (nnDetectionsMap == null && handle.vision().hasProcessor(VisionProcessorType.NN)) {
                nnDetectionsMap = new TreeMap<>();
            }
        }
    }

    @SuppressWarnings("unchecked")
    public <T extends VisionHandle> T getVisionHandleFor(VisionCameraID<T, ?> cameraID) {
        return (T) visionHandles.get(cameraID);
    }

    private static <T extends VisionTimestampedResult> void populateMap(NavigableMap<Double, VisionData<T>> map, Iterable<T> entries, VisionCameraID<?, ?> cameraID) {
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

                    if (settings.hasEnabledAll(VisionAprilTagFeature.LOCALIZATION)) {
                        populateMap(visionPoses, visionAprilTag3DProcessor.getRobotPoseObservation(), handle.identifier());
                    }

                    VisionAprilTagTracker recorder = visionAprilTag3DProcessor.getAprilTags();

                    if (settings.hasEnabledAny(VisionAprilTagFeature.BEST_DETECTION, VisionAprilTagFeature.ALL_DETECTIONS)) {
                        populateMap(aprilTagDetectionsMap, recorder, handle.identifier());
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
