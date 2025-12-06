package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class TagConstants {

    public static final Map<Integer, Pose3d> ANDYMARK_TAG_MAP = new HashMap<>();
    public static final Map<Integer, Pose3d> WELDED_TAG_MAP = new HashMap<>();
    public static final Map<Integer, Pose3d> PRACTICE_FIELD_MAP = new HashMap<>();

    static {
        PRACTICE_FIELD_MAP.put(
                1,
                new Pose3d(
                        new Translation3d(16.80024360671083, 0.6009765927805586, 1.5157529683921052),
                        new Rotation3d(new Quaternion(
                                0.42936835470163914, 0.000443900200277762, 0.019394917675954273, 0.9029210685889233))));

        PRACTICE_FIELD_MAP.put(
                2,
                new Pose3d(
                        new Translation3d(16.395598190798324, 7.5383133231920025, 1.5829352795985427),
                        new Rotation3d(new Quaternion(
                                -0.4160274140808628, 0.005394239745085365, 0.011803260754149256, 0.9092594656897075))));

        PRACTICE_FIELD_MAP.put(
                3,
                new Pose3d(
                        new Translation3d(11.59211713578528, 7.98789534074719, 1.297863853137649),
                        new Rotation3d(new Quaternion(
                                0.6828135811667768, 0.003616870672344038, -0.01727103141988132, -0.730379519903478))));

        PRACTICE_FIELD_MAP.put(
                4,
                new Pose3d(
                        new Translation3d(9.276079999999999, 6.137656, 1.8679160000000001),
                        new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.2588190451025207, 0))));

        PRACTICE_FIELD_MAP.put(
                5,
                new Pose3d(
                        new Translation3d(9.144480014842381, 1.8841544049516523, 1.918153842598806),
                        new Rotation3d(new Quaternion(
                                0.9691798437243901,
                                -0.004684909836960755,
                                0.24583928186841963,
                                0.015216097679951324))));

        PRACTICE_FIELD_MAP.put(
                6,
                new Pose3d(
                        new Translation3d(13.271693724431216, 2.9065803280063496, -0.051117151296432795),
                        new Rotation3d(new Quaternion(
                                0.8551861436480949, -0.029647953194374386, 0.03995993476436668, -0.5159271869143085))));

        PRACTICE_FIELD_MAP.put(
                7,
                new Pose3d(
                        new Translation3d(13.914583848300685, 4.03077420990323, 0.3115383991286884),
                        new Rotation3d(new Quaternion(
                                0.9995881177600249,
                                0.0013534869877074227,
                                -0.008362596485338264,
                                -0.027419516515941517))));

        PRACTICE_FIELD_MAP.put(
                8,
                new Pose3d(
                        new Translation3d(13.502313602009655, 4.72234428927136, 0.27692871636099176),
                        new Rotation3d(new Quaternion(
                                0.8715495143799722, 0.005842195681800816, 0.011468766950789628, 0.49013853155845527))));

        PRACTICE_FIELD_MAP.put(
                9,
                new Pose3d(
                        new Translation3d(12.643358, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));

        PRACTICE_FIELD_MAP.put(
                10,
                new Pose3d(
                        new Translation3d(12.273832167077469, 4.091771605915445, 0.3352365673122433),
                        new Rotation3d(new Quaternion(
                                0.002617613205513037,
                                0.013263673802789313,
                                -0.005772495335087313,
                                0.9998919448400242))));

        PRACTICE_FIELD_MAP.put(
                11,
                new Pose3d(
                        new Translation3d(12.67382480507537, 3.362881934137347, 0.30789147037949827),
                        new Rotation3d(new Quaternion(
                                -0.48647067465355437,
                                -0.002683090999419986,
                                -0.007875503174023147,
                                0.8736572898880662))));

        PRACTICE_FIELD_MAP.put(
                12,
                new Pose3d(
                        new Translation3d(0.851154, 0.65532, 1.4859),
                        new Rotation3d(new Quaternion(0.8910065241883679, 0, 0, 0.45399049973954675))));

        PRACTICE_FIELD_MAP.put(
                13,
                new Pose3d(
                        new Translation3d(0.851154, 7.3964799999999995, 1.4859),
                        new Rotation3d(new Quaternion(-0.8910065241883678, 0, 0, 0.45399049973954686))));

        PRACTICE_FIELD_MAP.put(
                14,
                new Pose3d(
                        new Translation3d(8.272272, 6.137656, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));

        PRACTICE_FIELD_MAP.put(
                15,
                new Pose3d(
                        new Translation3d(8.272272, 1.914906, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));

        PRACTICE_FIELD_MAP.put(
                16,
                new Pose3d(
                        new Translation3d(5.9875419999999995, -0.0038099999999999996, 1.30175),
                        new Rotation3d(new Quaternion(0.7071067811865476, 0, 0, 0.7071067811865476))));

        PRACTICE_FIELD_MAP.put(
                17,
                new Pose3d(
                        new Translation3d(4.073905999999999, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));

        PRACTICE_FIELD_MAP.put(
                18,
                new Pose3d(
                        new Translation3d(3.6576, 4.0259, 0.308102),
                        new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));

        PRACTICE_FIELD_MAP.put(
                19,
                new Pose3d(
                        new Translation3d(4.073905999999999, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));

        PRACTICE_FIELD_MAP.put(
                20,
                new Pose3d(
                        new Translation3d(4.904739999999999, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));

        PRACTICE_FIELD_MAP.put(
                21,
                new Pose3d(new Translation3d(5.321046, 4.0259, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));

        PRACTICE_FIELD_MAP.put(
                22,
                new Pose3d(
                        new Translation3d(4.904739999999999, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));

        ANDYMARK_TAG_MAP.put(
                1,
                new Pose3d(
                        new Translation3d(16.687292, 0.628142, 1.4859),
                        new Rotation3d(new Quaternion(0.4539904997395468, 0, 0, 0.8910065241883678))));
        ANDYMARK_TAG_MAP.put(
                2,
                new Pose3d(
                        new Translation3d(16.687292, 7.414259999999999, 1.4859),
                        new Rotation3d(new Quaternion(-0.45399049973954675, 0, 0, 0.8910065241883679))));
        ANDYMARK_TAG_MAP.put(
                3,
                new Pose3d(
                        new Translation3d(11.49096, 8.031733999999998, 1.30175),
                        new Rotation3d(new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476))));
        ANDYMARK_TAG_MAP.put(
                4,
                new Pose3d(
                        new Translation3d(9.276079999999999, 6.132575999999999, 1.8679160000000001),
                        new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        ANDYMARK_TAG_MAP.put(
                5,
                new Pose3d(
                        new Translation3d(9.276079999999999, 1.9098259999999998, 1.8679160000000001),
                        new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        ANDYMARK_TAG_MAP.put(
                6,
                new Pose3d(
                        new Translation3d(13.474446, 3.3012379999999997, 0.308102),
                        new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(
                7,
                new Pose3d(
                        new Translation3d(13.890498, 4.0208200000000005, 0.308102),
                        new Rotation3d(new Quaternion(1, 0, 0, 0))));
        ANDYMARK_TAG_MAP.put(
                8,
                new Pose3d(
                        new Translation3d(13.474446, 4.740402, 0.308102),
                        new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(
                9,
                new Pose3d(
                        new Translation3d(12.643358, 4.740402, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        ANDYMARK_TAG_MAP.put(
                10,
                new Pose3d(
                        new Translation3d(12.227305999999999, 4.0208200000000005, 0.308102),
                        new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        ANDYMARK_TAG_MAP.put(
                11,
                new Pose3d(
                        new Translation3d(12.643358, 3.3012379999999997, 0.308102),
                        new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        ANDYMARK_TAG_MAP.put(
                12,
                new Pose3d(
                        new Translation3d(0.8613139999999999, 0.628142, 1.4859),
                        new Rotation3d(new Quaternion(0.8910065241883679, 0, 0, 0.45399049973954675))));
        ANDYMARK_TAG_MAP.put(
                13,
                new Pose3d(
                        new Translation3d(0.8613139999999999, 7.414259999999999, 1.4859),
                        new Rotation3d(new Quaternion(-0.8910065241883678, 0, 0, 0.45399049973954686))));
        ANDYMARK_TAG_MAP.put(
                14,
                new Pose3d(
                        new Translation3d(8.272272, 6.132575999999999, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));
        ANDYMARK_TAG_MAP.put(
                15,
                new Pose3d(
                        new Translation3d(8.272272, 1.9098259999999998, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));
        ANDYMARK_TAG_MAP.put(
                16,
                new Pose3d(
                        new Translation3d(6.057646, 0.010667999999999999, 1.30175),
                        new Rotation3d(new Quaternion(0.7071067811865476, 0, 0, 0.7071067811865476))));
        ANDYMARK_TAG_MAP.put(
                17,
                new Pose3d(
                        new Translation3d(4.073905999999999, 3.3012379999999997, 0.308102),
                        new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        ANDYMARK_TAG_MAP.put(
                18,
                new Pose3d(
                        new Translation3d(3.6576, 4.0208200000000005, 0.308102),
                        new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        ANDYMARK_TAG_MAP.put(
                19,
                new Pose3d(
                        new Translation3d(4.073905999999999, 4.740402, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        ANDYMARK_TAG_MAP.put(
                20,
                new Pose3d(
                        new Translation3d(4.904739999999999, 4.740402, 0.308102),
                        new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        ANDYMARK_TAG_MAP.put(
                21,
                new Pose3d(
                        new Translation3d(5.321046, 4.0208200000000005, 0.308102),
                        new Rotation3d(new Quaternion(1, 0, 0, 0))));
        ANDYMARK_TAG_MAP.put(
                22,
                new Pose3d(
                        new Translation3d(4.904739999999999, 3.3012379999999997, 0.308102),
                        new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));

        WELDED_TAG_MAP.put(
                1,
                new Pose3d(
                        new Translation3d(16.697198, 0.65532, 1.4859),
                        new Rotation3d(new Quaternion(0.4539904997395468, 0, 0, 0.8910065241883678))));
        WELDED_TAG_MAP.put(
                2,
                new Pose3d(
                        new Translation3d(16.697198, 7.3964799999999995, 1.4859),
                        new Rotation3d(new Quaternion(-0.45399049973954675, 0, 0, 0.8910065241883679))));
        WELDED_TAG_MAP.put(
                3,
                new Pose3d(
                        new Translation3d(11.560809999999998, 8.05561, 1.30175),
                        new Rotation3d(new Quaternion(-0.7071067811865475, 0, 0, 0.7071067811865476))));
        WELDED_TAG_MAP.put(
                4,
                new Pose3d(
                        new Translation3d(9.276079999999999, 6.137656, 1.8679160000000001),
                        new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        WELDED_TAG_MAP.put(
                5,
                new Pose3d(
                        new Translation3d(9.276079999999999, 1.914906, 1.8679160000000001),
                        new Rotation3d(new Quaternion(0.9659258262890683, 0, 0.25881904510252074, 0))));
        WELDED_TAG_MAP.put(
                6,
                new Pose3d(
                        new Translation3d(13.474446, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(
                7,
                new Pose3d(new Translation3d(13.890498, 4.0259, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        WELDED_TAG_MAP.put(
                8,
                new Pose3d(
                        new Translation3d(13.474446, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(
                9,
                new Pose3d(
                        new Translation3d(12.643358, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        WELDED_TAG_MAP.put(
                10,
                new Pose3d(
                        new Translation3d(12.227305999999999, 4.0259, 0.308102),
                        new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        WELDED_TAG_MAP.put(
                11,
                new Pose3d(
                        new Translation3d(12.643358, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        WELDED_TAG_MAP.put(
                12,
                new Pose3d(
                        new Translation3d(0.851154, 0.65532, 1.4859),
                        new Rotation3d(new Quaternion(0.8910065241883679, 0, 0, 0.45399049973954675))));
        WELDED_TAG_MAP.put(
                13,
                new Pose3d(
                        new Translation3d(0.851154, 7.3964799999999995, 1.4859),
                        new Rotation3d(new Quaternion(-0.8910065241883678, 0, 0, 0.45399049973954686))));
        WELDED_TAG_MAP.put(
                14,
                new Pose3d(
                        new Translation3d(8.272272, 6.137656, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));
        WELDED_TAG_MAP.put(
                15,
                new Pose3d(
                        new Translation3d(8.272272, 1.914906, 1.8679160000000001),
                        new Rotation3d(new Quaternion(
                                5.914589856893349e-17,
                                -0.25881904510252074,
                                1.5848095757158825e-17,
                                0.9659258262890683))));
        WELDED_TAG_MAP.put(
                16,
                new Pose3d(
                        new Translation3d(5.9875419999999995, -0.0038099999999999996, 1.30175),
                        new Rotation3d(new Quaternion(0.7071067811865476, 0, 0, 0.7071067811865476))));
        WELDED_TAG_MAP.put(
                17,
                new Pose3d(
                        new Translation3d(4.073905999999999, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.4999999999999998, 0, 0, 0.8660254037844387))));
        WELDED_TAG_MAP.put(
                18,
                new Pose3d(
                        new Translation3d(3.6576, 4.0259, 0.308102),
                        new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))));
        WELDED_TAG_MAP.put(
                19,
                new Pose3d(
                        new Translation3d(4.073905999999999, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.5000000000000001, 0, 0, 0.8660254037844386))));
        WELDED_TAG_MAP.put(
                20,
                new Pose3d(
                        new Translation3d(4.904739999999999, 4.745482, 0.308102),
                        new Rotation3d(new Quaternion(0.8660254037844387, 0, 0, 0.49999999999999994))));
        WELDED_TAG_MAP.put(
                21,
                new Pose3d(new Translation3d(5.321046, 4.0259, 0.308102), new Rotation3d(new Quaternion(1, 0, 0, 0))));
        WELDED_TAG_MAP.put(
                22,
                new Pose3d(
                        new Translation3d(4.904739999999999, 3.3063179999999996, 0.308102),
                        new Rotation3d(new Quaternion(-0.8660254037844387, 0, 0, 0.49999999999999994))));
    }

    public static Optional<Pose3d> getTagPose(int tagID) {
        return Optional.ofNullable(PRACTICE_FIELD_MAP.get(tagID));
    }
}
