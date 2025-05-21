package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.BitSet;
import java.util.Optional;

public class VisionAprilTagRecorder implements LoggableInputs {
    private static final int TAG_COUNT = VisionUtil.APRIL_TAG_FIELD_LAYOUT.getTags().size();
    private final VisionAprilTag3DAutoLogged[] tagData = new VisionAprilTag3DAutoLogged[TAG_COUNT];
    private final BitSet tagUpdatedBitSet = new BitSet(TAG_COUNT);
    private int bestTagID = -1;

    public void record(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (fiducialID < 0 || fiducialID >= tagData.length) return;

        tagUpdatedBitSet.set(fiducialID);

        if (tagData[--fiducialID] == null) {
            tagData[fiducialID] = new VisionAprilTag3DAutoLogged();
        }

        if (tagData[fiducialID].timestamp() > timestamp) return;

        tagData[fiducialID].setFrom(fiducialID, robotToTargetPose, ambiguity, timestamp);
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

    @Override
    public void toLog(LogTable table) {
        table.put("BestTagID", bestTagID);

        for (int i = 0; i < tagData.length; i++) {
            if (tagUpdatedBitSet.get(i)) {
                table.put("Tag" + i, tagData[i]);
            }
        }

        tagUpdatedBitSet.clear();
    }

    @Override
    public void fromLog(LogTable table) {
        VisionAprilTag3DAutoLogged tagDataRead = new VisionAprilTag3DAutoLogged();

        bestTagID = table.get("BestTagID", bestTagID);

        for (int i = 0; i < tagData.length; i++) {
            if (tagData[i] == null) {
                tagData[i] = new VisionAprilTag3DAutoLogged();
            }

            table.get("Tag" + i, tagDataRead);

            if (tagDataRead.robotToTargetPose != null) {
                tagData[i].from(tagDataRead);
            }
        }
    }
}
