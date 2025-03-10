package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagPose {
    private final Pose2d estimatedRobotPose;
    private final int numTags;
    private final double timestamp;

    public AprilTagPose(Pose2d estimatedRobotPose, int numTags, double timestamp) {
        this.estimatedRobotPose = estimatedRobotPose;
        this.numTags = numTags;
        this.timestamp = timestamp;
    }

    public int getNumTags() {
        return numTags;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }
}
