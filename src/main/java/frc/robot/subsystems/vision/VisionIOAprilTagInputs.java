package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.data.VisionRobotPose;
import frc.robot.subsystems.vision.data.apriltag.VisionAprilTagTracker;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class VisionIOAprilTagInputs extends VisionIOProcessorInputs {
    public VisionRobotPose[] estimatedPoses;
    public VisionAprilTagTracker aprilTags;
}
