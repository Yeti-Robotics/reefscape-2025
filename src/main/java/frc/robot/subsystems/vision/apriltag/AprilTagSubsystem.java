package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

public interface AprilTagSubsystem {
    Optional<AprilTagResults> getResults();
    Optional<AprilTagPose> getEstimatedPose();

    default Optional<AprilTagDetection> findDetection(int fiducialId) {
        Optional<AprilTagResults> results = getResults();

        if (results.isEmpty()) return Optional.empty();

        for (AprilTagDetection tag : results.get().getResults()) {
            if (tag.getFiducialID() == fiducialId) {
                return Optional.of(tag);
            }
        }

        return Optional.empty();
    }

    void filterFiducialIDs(int... ids);
}
