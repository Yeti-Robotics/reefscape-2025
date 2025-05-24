package frc.robot.subsystems.vision.data.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.BitSet;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Optional;

public class VisionAprilTagTracker implements LoggableInputs, Iterable<VisionAprilTag3D> {
    private static final int TAG_COUNT = VisionUtil.APRIL_TAG_FIELD_LAYOUT.getTags().size();
    private static final String TAGS_KEY = "Tags";
    private static final String BEST_TAG_KEY = "BestTagID";

    private final VisionAprilTag3D[] tagData = new VisionAprilTag3D[TAG_COUNT];

    private final BitSet tagUnreadBitset = new BitSet(TAG_COUNT);

    private int bestTagID = -1;

    public boolean addObservation(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (fiducialID < 1 || fiducialID >= tagData.length) return false;

        int fiducialIndex = fiducialID - 1;

        if (tagData[fiducialIndex] == null) {
            tagData[fiducialIndex] = new VisionAprilTag3D();
        } else if (tagData[fiducialIndex].timestamp() > timestamp) return false;

        tagData[fiducialIndex].setFrom(fiducialID, robotToTargetPose, ambiguity, timestamp);

        tagUnreadBitset.set(fiducialIndex);

        return true;
    }

    public void addBestObservation(int fiducialID, Pose2d robotToTargetPose, double ambiguity, double timestamp) {
        if (addObservation(fiducialID, robotToTargetPose, ambiguity, timestamp)) {
            bestTagID = fiducialID;
        }
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
    }

    @Override
    public void fromLog(LogTable table) {
        bestTagID = table.get(BEST_TAG_KEY, bestTagID);
        VisionAprilTag3D[] loggedTagData = table.get(TAGS_KEY, VisionAprilTag3D.struct, tagData);

        for (VisionAprilTag3D aprilTag3D : loggedTagData) {
            addObservation(aprilTag3D.fiducialID, aprilTag3D.robotToTargetPose, aprilTag3D.ambiguity, aprilTag3D.timestamp);
        }
    }

    @Override
    public Iterator<VisionAprilTag3D> iterator() {
        return new Iterator<>() {
            private int nextBit = tagUnreadBitset.nextSetBit(0);

            @Override
            public boolean hasNext() {
                return nextBit >= 0;
            }

            @Override
            public VisionAprilTag3D next() {
                if (!hasNext()) {
                    throw new NoSuchElementException("No more unread tags");
                }

                VisionAprilTag3D tag = tagData[nextBit];
                tagUnreadBitset.clear(nextBit);
                nextBit = tagUnreadBitset.nextSetBit(nextBit + 1);
                return tag;
            }
        };
    }
}
