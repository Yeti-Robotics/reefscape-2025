package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.data.VisionTimestampedResult;

public class VisionAprilTag3D implements VisionTimestampedResult {
    private int fiducialID;
    private Pose2d robotToTargetPose;
    private double ambiguity;
    private double timestamp;

    public VisionAprilTag3D(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        setFrom(fiducialID, robotToTargetPose, ambiguity, timestamp);
    }

    public int fiducialID() {
        return fiducialID;
    }

    public Pose2d robotToTargetPose() {
        return robotToTargetPose;
    }

    public double ambiguity() {
        return ambiguity;
    }

    public double timestamp() {
        return timestamp;
    }

    public void setFrom(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        this.fiducialID = fiducialID;
        this.robotToTargetPose = robotToTargetPose;
        this.ambiguity = ambiguity;
        this.timestamp = timestamp;
    }

    public VisionAprilTag3D copy() {
        return new VisionAprilTag3D(fiducialID, robotToTargetPose, ambiguity, timestamp);
    }
}

