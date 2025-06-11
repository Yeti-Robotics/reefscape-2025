package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.data.nn.VisionNNDetection;
import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.io.api.VisionHandle;
import frc.robot.subsystems.vision.io.api.processor.VisionNNProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTag3DProcessor;
import frc.robot.subsystems.vision.io.api.processor.apriltag.VisionAprilTagSettings;
import org.littletonrobotics.junction.Logger;

public class VisionLog {
    private static final VisionIOAprilTagInputsAutoLogged poseInputsAutoLogged = new VisionIOAprilTagInputsAutoLogged();
    private static final VisionIONNDetectionInputsAutoLogged nnInputsAutoLogged = new VisionIONNDetectionInputsAutoLogged();

    public static void logProcessor(VisionHandle<?> handle, VisionAprilTag3DProcessor aprilTag3DProcessor) {
        VisionAprilTagSettings settings = aprilTag3DProcessor.getSettings();

        poseInputsAutoLogged.latencyMs = aprilTag3DProcessor.latencyMs();

        if (settings.hasEnabledAll(VisionAprilTagSettings.VisionAprilTagFeature.LOCALIZATION)) {
            poseInputsAutoLogged.estimatedPoses = aprilTag3DProcessor.getRobotPoseObservation().toArray(VisionRobotPose[]::new);
        }

        if (settings.hasEnabledAny(VisionAprilTagSettings.VisionAprilTagFeature.BEST_DETECTION, VisionAprilTagSettings.VisionAprilTagFeature.ALL_DETECTIONS)) {
           poseInputsAutoLogged.aprilTags = aprilTag3DProcessor.getAprilTags();
        }

        Logger.processInputs(handle.identifier().cameraName, poseInputsAutoLogged);
    }

    public static void logProcessor(VisionHandle<?> handle, VisionNNProcessor visionNNProcessor) {
        nnInputsAutoLogged.latencyMs = visionNNProcessor.latencyMs();
        nnInputsAutoLogged.detections = visionNNProcessor.getLatestNNDetections().toArray(VisionNNDetection[]::new);

        Logger.processInputs(handle.identifier().cameraName, nnInputsAutoLogged);
    }

    public static void logHandle(VisionHandle<?> handle) {
        Logger.recordOutput(handle.identifier().cameraName + "/Connected", handle.isConnected());
    }
}
