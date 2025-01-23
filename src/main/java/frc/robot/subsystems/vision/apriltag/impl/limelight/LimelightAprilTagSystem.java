package frc.robot.subsystems.vision.apriltag.impl.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagResults;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.util.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightAprilTagSystem extends SubsystemBase implements AprilTagSubsystem {
    private final String limelightName;
    private AprilTagResults aprilTagResults;

    public LimelightAprilTagSystem(String limelightName) {
        this.limelightName = limelightName;
    }

    private LimelightHelpers.LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    private AprilTagDetection mapToDetection(LimelightHelpers.LimelightTarget_Fiducial detection) {
        return new AprilTagDetection(
                detection.fiducialID,
                detection.getRobotPose_FieldSpace2D(),
                detection.getTargetPose_RobotSpace2D(),
                0 // we can trust MegaTag2, as it eliminates pose ambiguity
        );
    }

    @Override
    public void periodic() {
        double yawPlaceholder = 0;
        LimelightHelpers.SetRobotOrientation(limelightName, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder, yawPlaceholder);

        LimelightHelpers.LimelightResults results = getLatestResults();

        if (!results.valid) {
            aprilTagResults = null;
            return;
        }

        List<AprilTagDetection> aprilTagDetections = new ArrayList<>(getLatestResults().targets_Fiducials.length);

        for (LimelightHelpers.LimelightTarget_Fiducial aprilTag : results.targets_Fiducials) {
            aprilTagDetections.add(mapToDetection(aprilTag));
        }

        aprilTagResults = new AprilTagResults(results.timestamp_LIMELIGHT_publish, results.latency_pipeline, aprilTagDetections);
    }

    @Override
    public Optional<AprilTagResults> getResults() {
        return Optional.ofNullable(aprilTagResults);
    }

    public void filterFiducialIDs(int... fiducialIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, fiducialIDs);
    }
}
