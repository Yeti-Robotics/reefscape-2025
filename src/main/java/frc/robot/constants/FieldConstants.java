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
            blueCenterFaces[0] = APRIL_TAG_FIELD_LAYOUT.getTagPose(18).get().toPose2d();
            blueCenterFaces[1] = APRIL_TAG_FIELD_LAYOUT.getTagPose(19).get().toPose2d();
            blueCenterFaces[2] = APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get().toPose2d();
            blueCenterFaces[3] = APRIL_TAG_FIELD_LAYOUT.getTagPose(21).get().toPose2d();
            blueCenterFaces[4] = APRIL_TAG_FIELD_LAYOUT.getTagPose(22).get().toPose2d();
            blueCenterFaces[5] = APRIL_TAG_FIELD_LAYOUT.getTagPose(17).get().toPose2d();

            redCenterFaces[0] = APRIL_TAG_FIELD_LAYOUT.getTagPose(7).get().toPose2d();
            redCenterFaces[1] = APRIL_TAG_FIELD_LAYOUT.getTagPose(8).get().toPose2d();
            redCenterFaces[2] = APRIL_TAG_FIELD_LAYOUT.getTagPose(9).get().toPose2d();
            redCenterFaces[3] = APRIL_TAG_FIELD_LAYOUT.getTagPose(10).get().toPose2d();
            redCenterFaces[4] = APRIL_TAG_FIELD_LAYOUT.getTagPose(11).get().toPose2d();
            redCenterFaces[5] = APRIL_TAG_FIELD_LAYOUT.getTagPose(6).get().toPose2d();

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
                Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
                Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
                Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
                for (var level : ReefLevel.values()) {
                    Pose2d poseDirection =
                            new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    var rightBranchPose =
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    Rotation2d.kZero))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    Rotation2d.kZero))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians()));
                    var leftBranchPose =
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    Rotation2d.kZero))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    Rotation2d.kZero))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians()));

                    fillRight.put(level, rightBranchPose);
                    fillLeft.put(level, leftBranchPose);
                    fillRight2d.put(level, rightBranchPose.toPose2d());
                    fillLeft2d.put(level, leftBranchPose.toPose2d());
                }
                branchPositions.add(fillRight);
                branchPositions.add(fillLeft);
                branchPositions2d.add(fillRight2d);
                branchPositions2d.add(fillLeft2d);
            }
        }
    }
}
