package frc.robot.subsystems.vision.apriltag;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public interface AprilTagSubsystem {
    Optional<AprilTagResults> getResults();
    Optional<AprilTagPose> getEstimatedPose();

    default Optional<AprilTagDetection> findDetection(int fiducialId) {
        return getResults()
                .map(AprilTagResults::getResults)
                .flatMap(results -> results.stream()
                        .filter(t -> t.getFiducialID() == fiducialId)
                        .findFirst());
    }

    default List<AprilTagDetection> findDetections(int ...ids) {
        Optional<AprilTagResults> results = getResults();

        return results.map(aprilTagResults -> aprilTagResults.getResults().stream()
                .filter(tag -> Arrays.stream(ids).anyMatch(id -> id == tag.getFiducialID()))
                .collect(Collectors.toList())).orElseGet(List::of);
    }

    void onlyTrackTags(int... ids);
}
