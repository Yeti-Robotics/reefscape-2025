package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;

public class FieldConstants {
    public static final double FIELD_LENGTH = 16.5354;

    // Copyright (c) 2025 FRC 6328
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by an MIT-style
    // license that can be found in the LICENSE file at
    // the root directory of this project.

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final double fieldWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

    public enum ReefLevel {
        L1(Units.inchesToMeters(25.0), 0),
        L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
        L4(Units.inchesToMeters(72), -90);

        ReefLevel(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // Degrees
        }

        public static ReefLevel fromLevel(int level) {
            return Arrays.stream(values())
                    .filter(height -> height.ordinal() == level)
                    .findFirst()
                    .orElse(L4);
        }

        public final double height;
        public final double pitch;
    }

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] blueCenterFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final Pose2d[] redCenterFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefLevel, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise
        public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

        static {
            // Initialize faces
            blueCenterFaces[0] = TagConstants.Welded.TAG_18_POSE.toPose2d();
            blueCenterFaces[1] = TagConstants.Welded.TAG_19_POSE.toPose2d();
            blueCenterFaces[2] = TagConstants.Welded.TAG_20_POSE.toPose2d();
            blueCenterFaces[3] = TagConstants.Welded.TAG_21_POSE.toPose2d();
            blueCenterFaces[4] = TagConstants.Welded.TAG_22_POSE.toPose2d();
            blueCenterFaces[5] = TagConstants.Welded.TAG_17_POSE.toPose2d();

            redCenterFaces[0] = TagConstants.Welded.TAG_7_POSE.toPose2d();
            redCenterFaces[1] = TagConstants.Welded.TAG_8_POSE.toPose2d();
            redCenterFaces[2] = TagConstants.Welded.TAG_9_POSE.toPose2d();
            redCenterFaces[3] = TagConstants.Welded.TAG_10_POSE.toPose2d();
            redCenterFaces[4] = TagConstants.Welded.TAG_11_POSE.toPose2d();
            redCenterFaces[5] = TagConstants.Welded.TAG_6_POSE.toPose2d();
        }
    }
}
