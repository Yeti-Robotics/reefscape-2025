package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.BitSet;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Optional;

public class VisionAprilTagTracker implements LoggableInputs {
    private static final int TAG_COUNT = VisionUtil.APRIL_TAG_FIELD_LAYOUT.getTags().size();
    private static final String TAGS_KEY = "Tags";
    private static final String BEST_TAG_KEY = "BestTagID";

    private final VisionAprilTag3D[] tagData = new VisionAprilTag3D[TAG_COUNT];

    private final BitSet tagUnreadBitset = new BitSet(TAG_COUNT);

    private int bestTagID = -1;

    public boolean addObservation(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (fiducialID < 0 || fiducialID >= tagData.length) return false;

        int fiducialIndex = fiducialID - 1;
        VisionAprilTag3D tag = tagData[fiducialIndex];

        if (tag == null) {
            tagData[fiducialIndex] = tag = new VisionAprilTag3D();
        } else if (tag.timestamp > timestamp) return false;

        tag.setData(fiducialID, robotToTargetPose, ambiguity, timestamp);
        tagUnreadBitset.set(fiducialIndex);
        return true;
    }

    public boolean addBestObservation(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (addObservation(fiducialID, robotToTargetPose, ambiguity, timestamp)) {
            bestTagID = fiducialID;
            return true;
        }

        return false;
    }

    public Optional<VisionAprilTag3D> getBestTag() {
        return getTag(bestTagID);
    }

    public Optional<VisionAprilTag3D> getTag(int fiducialID) {
        return fiducialID < 0 || fiducialID >= tagData.length ? Optional.empty() : Optional.ofNullable(tagData[fiducialID - 1]);
    }

    @Override
    public void toLog(LogTable table) {
        table.put(BEST_TAG_KEY, bestTagID);

        int tagsToLog = tagUnreadBitset.cardinality();

        VisionAprilTag3D[] loggedTags = new VisionAprilTag3D[tagsToLog];
        if (tagsToLog > 0) {
            int arrIndex = 0;

            for (int i = tagUnreadBitset.nextSetBit(0); i >= 0; i = tagUnreadBitset.nextSetBit(i + 1)) {
                loggedTags[arrIndex++] = tagData[i];
            }
        }

        table.put(TAGS_KEY, VisionAprilTag3D.struct, loggedTags);
        tagUnreadBitset.clear();
    }

    @Override
    public void fromLog(LogTable table) {
        bestTagID = table.get(BEST_TAG_KEY, bestTagID);
        VisionAprilTag3D[] loggedTagData = table.get(TAGS_KEY, VisionAprilTag3D.struct, tagData);

        for (VisionAprilTag3D aprilTag3D : loggedTagData) {
            addObservation(aprilTag3D.fiducialID, aprilTag3D.robotToTargetPose, aprilTag3D.ambiguity, aprilTag3D.timestamp);
        }
    }
}
