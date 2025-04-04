package frc.robot.constants;

import edu.wpi.first.math.geometry.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class TagConstants {

    public static final Map<Integer, Pose3d> ANDYMARK_TAG_MAP = new HashMap<>();
    public static final Map<Integer, Pose3d> WELDED_TAG_MAP = new HashMap<>();

    static {

        ANDYMARK_TAG_MAP.put(1, new Pose3d(new Translation3d(16.687292, 0.628142, 1.4859), new Rotation3d(new Quaternion(0.4539904997395468, 0, 0, 0.8910065241883678))));
        ANDYMARK_TAG_MAP.put(2, new Pose3d(new Translation3d(16.687292, 7.414259999999999, 1.4859), new Rotation3d(new Quaternion(-0.45399049973954675, 0, 0, 0.8910065241883679))));
        ANDYMARK_TAG_MAP.put(3, new Pose3d(new Translation3d(11.49096, 8.031733999999998, 1.30175), new Rotation3d(new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476))));
        ANDYMARK_TAG_MAP.put(4, new Pose3d(new Translation3d(9.276079999999999, 6.132575999999999, 1.8679160000000001), new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        ANDYMARK_TAG_MAP.put(5, new Pose3d(new Translation3d(9.276079999999999, 1.9098259999999998, 1.8679160000000001), new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        ANDYMARK_TAG_MAP.put(6, new Pose3d(new Translation3d(13.474446, 3.3012379999999997, 0.308102), new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(7, new Pose3d(new Translation3d(13.890498, 4.0208200000000005, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        ANDYMARK_TAG_MAP.put(8, new Pose3d(new Translation3d(13.474446, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(9, new Pose3d(new Translation3d(12.643358, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        ANDYMARK_TAG_MAP.put(10, new Pose3d(new Translation3d(12.227305999999999, 4.0208200000000005, 0.308102), new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        ANDYMARK_TAG_MAP.put(11, new Pose3d(new Translation3d(12.643358, 3.3012379999999997, 0.308102), new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        ANDYMARK_TAG_MAP.put(12, new Pose3d(new Translation3d(0.8613139999999999, 0.628142, 1.4859), new Rotation3d(new Quaternion(0.8910065241883679, 0, 0, 0.45399049973954675))));
        ANDYMARK_TAG_MAP.put(13, new Pose3d(new Translation3d(0.8613139999999999, 7.414259999999999, 1.4859), new Rotation3d(new Quaternion(-0.8910065241883678, 0, 0, 0.45399049973954686))));
        ANDYMARK_TAG_MAP.put(14, new Pose3d(new Translation3d(8.272272, 6.132575999999999, 1.8679160000000001), new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))));
        ANDYMARK_TAG_MAP.put(15, new Pose3d(new Translation3d(8.272272, 1.9098259999999998, 1.8679160000000001), new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))));
        ANDYMARK_TAG_MAP.put(16, new Pose3d(new Translation3d(6.057646, 0.010667999999999999, 1.30175), new Rotation3d(new Quaternion(0.7071067811865476, 0, 0, 0.7071067811865476))));
        ANDYMARK_TAG_MAP.put(17, new Pose3d(new Translation3d(4.073905999999999, 3.3012379999999997, 0.308102), new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        ANDYMARK_TAG_MAP.put(18, new Pose3d(new Translation3d(3.6576, 4.0208200000000005, 0.308102), new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        ANDYMARK_TAG_MAP.put(19, new Pose3d(new Translation3d(4.073905999999999, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        ANDYMARK_TAG_MAP.put(20, new Pose3d(new Translation3d(4.904739999999999, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(21, new Pose3d(new Translation3d(5.321046, 4.0208200000000005, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        ANDYMARK_TAG_MAP.put(22, new Pose3d(new Translation3d(4.904739999999999, 3.3012379999999997, 0.308102), new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));

        WELDED_TAG_MAP.put(1, new Pose3d(new Translation3d(16.697198, 0.65532, 1.4859), new Rotation3d(new Quaternion(0.4539904997395468, 0, 0, 0.8910065241883678))));
        WELDED_TAG_MAP.put(2, new Pose3d(new Translation3d(16.697198, 7.3964799999999995, 1.4859), new Rotation3d(new Quaternion(-0.45399049973954675, 0, 0, 0.8910065241883679))));
        WELDED_TAG_MAP.put(3, new Pose3d(new Translation3d(11.560809999999998, 8.05561, 1.30175), new Rotation3d(new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476))));
        WELDED_TAG_MAP.put(4, new Pose3d(new Translation3d(9.276079999999999, 6.137656, 1.8679160000000001), new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        WELDED_TAG_MAP.put(5, new Pose3d(new Translation3d(9.276079999999999, 1.914906, 1.8679160000000001), new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        WELDED_TAG_MAP.put(6, new Pose3d(new Translation3d(13.474446, 3.3063179999999996, 0.308102), new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(7, new Pose3d(new Translation3d(13.890498, 4.0259, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        WELDED_TAG_MAP.put(8, new Pose3d(new Translation3d(13.474446, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(9, new Pose3d(new Translation3d(12.643358, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        WELDED_TAG_MAP.put(10, new Pose3d(new Translation3d(12.227305999999999, 4.0259, 0.308102), new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        WELDED_TAG_MAP.put(11, new Pose3d(new Translation3d(12.643358, 3.3063179999999996, 0.308102), new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        WELDED_TAG_MAP.put(12, new Pose3d(new Translation3d(0.851154, 0.65532, 1.4859), new Rotation3d(new Quaternion(0.8910065241883679, 0, 0, 0.45399049973954675))));
        WELDED_TAG_MAP.put(13, new Pose3d(new Translation3d(0.851154, 7.3964799999999995, 1.4859), new Rotation3d(new Quaternion(-0.8910065241883678, 0, 0, 0.45399049973954686))));
        WELDED_TAG_MAP.put(14, new Pose3d(new Translation3d(8.272272, 6.137656, 1.8679160000000001), new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))));
        WELDED_TAG_MAP.put(15, new Pose3d(new Translation3d(8.272272, 1.914906, 1.8679160000000001), new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))));
        WELDED_TAG_MAP.put(16, new Pose3d(new Translation3d(5.9875419999999995, -0.0038099999999999996, 1.30175), new Rotation3d(new Quaternion(0.7071067811865476, 0, 0, 0.7071067811865476))));
        WELDED_TAG_MAP.put(17, new Pose3d(new Translation3d(4.073905999999999, 3.3063179999999996, 0.308102), new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        WELDED_TAG_MAP.put(18, new Pose3d(new Translation3d(3.6576, 4.0259, 0.308102), new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        WELDED_TAG_MAP.put(19, new Pose3d(new Translation3d(4.073905999999999, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        WELDED_TAG_MAP.put(20, new Pose3d(new Translation3d(4.904739999999999, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(21, new Pose3d(new Translation3d(5.321046, 4.0259, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        WELDED_TAG_MAP.put(22, new Pose3d(new Translation3d(4.904739999999999, 3.3063179999999996, 0.308102), new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
    }

    public static Optional<Pose3d> getTagPose(int tagID) {
        return Optional.ofNullable(WELDED_TAG_MAP.get(tagID));
    }
}
