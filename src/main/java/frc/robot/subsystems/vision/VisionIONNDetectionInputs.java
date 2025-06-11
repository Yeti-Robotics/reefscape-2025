package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.data.nn.VisionNNDetection;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionIONNDetectionInputs extends VisionIOProcessorInputs {
    public VisionNNDetection[] detections;
}
