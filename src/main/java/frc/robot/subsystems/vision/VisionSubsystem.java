package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.data.VisionData;
import frc.robot.subsystems.vision.data.VisionNNDetection;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.VisionTimestampedResult;
import frc.robot.subsystems.vision.io.api.*;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings.VisionAprilTagFeature;
import frc.robot.subsystems.vision.io.api.processor.*;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.impl.limelight.LimelightBuilder;
import frc.robot.subsystems.vision.io.impl.photon.PhotonVisionBuilder;

import java.util.*;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private interface VisionBuilderSupplier<T extends VisionHandleBuilder> {
        T createBuilder(VisionCameraID cameraID, Supplier<Rotation2d> rotationSupplier, Transform3d robotToCameraTransform);
    }

    private final Map<VisionCameraID, VisionHandle> visionHandles = new HashMap<>();

    // apriltag data/configs
    private final Supplier<Rotation2d> drivetrainRotation;
    private final NavigableMap<Double, VisionData<VisionRobotPose>> visionPoses = new TreeMap<>();

    // neural net detections
    private NavigableMap<Double, VisionData<VisionNNDetection>> nnDetectionsMap = null;

    public VisionSubsystem(Supplier<Rotation2d> drivetrainRotation) {
        this.drivetrainRotation = drivetrainRotation;
    }


    private <T extends VisionHandleBuilder> T createCamera(VisionCameraID cameraID,
                                                           VisionBuilderSupplier<T> supplier,
                                                           Transform3d robotToCameraTransform,
                                                           VisionCameraID.VisionType type) {
        if (cameraID.visionType != type) {
            throw new IllegalArgumentException("Camera " + cameraID.cameraName + " of type " + cameraID.visionType + " is not a " + type + " camera");
        }

        return supplier.createBuilder(cameraID, drivetrainRotation, robotToCameraTransform);
    }

    public PhotonVisionBuilder createPhotonVisionCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        return createCamera(cameraID, PhotonVisionBuilder::createBuilder, robotToCameraTransform, VisionCameraID.VisionType.PHOTONVISION);
    }

    public LimelightBuilder createLimelightCamera(VisionCameraID cameraID, Transform3d robotToCameraTransform) {
        return createCamera(cameraID, LimelightBuilder::new, robotToCameraTransform, VisionCameraID.VisionType.LIMELIGHT_MEGATAG_2);
    }

    public Optional<VisionData<VisionRobotPose>> pollVisionPoseUpdate() {
        Map.Entry<Double, VisionData<VisionRobotPose>> entry = visionPoses.pollLastEntry();
        return entry == null ? Optional.empty() : Optional.of(entry.getValue());
    }


    public void addVisionHandle(VisionHandle... handles) {
        for (VisionHandle handle : handles) {
            visionHandles.put(handle.identifier(), handle);

            if (nnDetectionsMap == null && handle.vision().hasProcessor(VisionProcessorType.NN)) {
                nnDetectionsMap = new TreeMap<>();
            }
        }
    }

    private static <T extends VisionTimestampedResult> void populateMap(NavigableMap<Double, VisionData<T>> map, Iterable<T> entries, VisionCameraID cameraID) {
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
            VisionLog.logHandle(handle);

            VisionProcessorPipelineManager processorManager = handle.vision();

            if (processorManager.activeVisionProcessorType() == VisionProcessorType.APRILTAG_3D) {
                Optional<VisionAprilTag3DProcessor> aprilTagProcessor = fetchProcessorForHandle(handle, VisionProcessorType.APRILTAG_3D);

                if (aprilTagProcessor.isPresent()) {
                    VisionAprilTag3DProcessor visionAprilTag3DProcessor = aprilTagProcessor.get();
                    VisionAprilTagSettings settings = visionAprilTag3DProcessor.getSettings();

                    if (settings.hasEnabledAll(VisionAprilTagFeature.LOCALIZATION)) {
                        populateMap(visionPoses, visionAprilTag3DProcessor.getRobotPoseObservation(), handle.identifier());
                    }

                    VisionLog.logProcessor(handle, visionAprilTag3DProcessor);
                }
            } else if (processorManager.activeVisionProcessorType() == VisionProcessorType.NN) {
                Optional<VisionNNProcessor> nnProcessor = fetchProcessorForHandle(handle, VisionProcessorType.NN);

                if (nnProcessor.isPresent()) {
                    VisionNNProcessor visionNNProcessor = nnProcessor.get();

                    if (nnDetectionsMap != null) {
                        populateMap(nnDetectionsMap, visionNNProcessor.getLatestNNDetections(), handle.identifier());
                    }

                    VisionLog.logProcessor(handle, visionNNProcessor);
                }
            }
        }
    }
}
