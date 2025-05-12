package frc.robot.subsystems.vision.data;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionAprilTag(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp)
        implements VisionTimestampedResult {}
