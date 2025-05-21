package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.data.VisionTimestampedResult;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class VisionAprilTag3D implements VisionTimestampedResult {
    protected int fiducialID;
    protected Pose2d robotToTargetPose;
    protected double ambiguity;
    protected double timestamp;

    public VisionAprilTag3D(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        setFrom(fiducialID, robotToTargetPose, ambiguity, timestamp);
    }

    public VisionAprilTag3D() {}

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

    public VisionAprilTag3D from(VisionAprilTag3D visionAprilTag3D) {
        this.fiducialID = visionAprilTag3D.fiducialID;
        this.robotToTargetPose = visionAprilTag3D.robotToTargetPose;
        this.ambiguity = visionAprilTag3D.ambiguity;
        this.timestamp = visionAprilTag3D.timestamp;
        return this;
    }
}

