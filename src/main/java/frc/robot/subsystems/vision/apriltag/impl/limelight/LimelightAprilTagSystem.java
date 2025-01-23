package frc.robot.subsystems.vision.apriltag.impl.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private final String limelightName;
    private AprilTagResults aprilTagResults;
    private LimelightHelpers.PoseEstimate estimatedRobotPose;

    public LimelightAprilTagSystem(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void periodic() {
        double yawPlaceholder = 0; // need to add drivetrain
        LimelightHelpers.SetRobotOrientation(limelightName, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder);
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);


        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

        if (!results.valid) {
            aprilTagResults = null;
            return;
        }

        List<AprilTagDetection> aprilTagDetections = new ArrayList<>(results.targets_Fiducials.length);

        for (LimelightHelpers.LimelightTarget_Fiducial aprilTag : results.targets_Fiducials) {
            aprilTagDetections.add(new AprilTagDetection(
                    aprilTag.fiducialID,
                    aprilTag.getRobotPose_FieldSpace2D(),
                    aprilTag.getTargetPose_RobotSpace2D(),
                    0 // we can trust MegaTag2, as it eliminates pose ambiguity
            ));
        }

        aprilTagResults = new AprilTagResults(results.timestamp_LIMELIGHT_publish, results.latency_pipeline, aprilTagDetections);
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        return Optional.ofNullable(aprilTagResults);
    }

    @Override
    public Optional<AprilTagPose> getEstimatedPose() {
        return Optional.ofNullable(estimatedRobotPose)
                .map(e -> new AprilTagPose(e.pose, e.tagCount, e.timestampSeconds));
    }

    public void filterFiducialIDs(int... fiducialIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, fiducialIDs);
    }
}
