package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionUtil;

import java.util.Optional;

public class VisionAprilTagRecorder {
    private static final int INVALID_TAG_ID = -1;

    private final VisionAprilTag3D[] tagData = new VisionAprilTag3D[VisionUtil.APRIL_TAG_FIELD_LAYOUT.getTags().size()];
    private int bestTagID = -1;

    public void record(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (fiducialID < 0 || fiducialID >= tagData.length) return;

        fiducialID = fiducialID - 1;

        if (tagData[fiducialID] == null) {
            tagData[fiducialID] = new VisionAprilTag3D(fiducialID, robotToTargetPose, ambiguity, timestamp);
        } else {
            VisionAprilTag3D currentTag = tagData[fiducialID];

            if (currentTag.timestamp() > timestamp) return;

            tagData[fiducialID].setFrom(fiducialID, robotToTargetPose, ambiguity, timestamp);
        }
    }

    public void setBestTag(int fiducialID) {
        bestTagID = fiducialID;
    }

    public Optional<VisionAprilTag3D> getBestTag() {
        return (bestTagID < 0 || bestTagID >= tagData.length) || tagData[bestTagID - 1] == null ? Optional.empty() : Optional.of(tagData[bestTagID - 1]);
    }

    public Optional<VisionAprilTag3D> getTag(int fiducialID) {
        return fiducialID < 0 || fiducialID >= tagData.length ? Optional.empty() : Optional.ofNullable(tagData[fiducialID - 1]);
    }
}
