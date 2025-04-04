package frc.robot.subsystems.vision.apriltag;

import java.util.*;

public interface AprilTagSubsystem {
    Optional<AprilTagResults> getResults();

    List<AprilTagPose> getEstimatedPose();

    Optional<AprilTagDetection> getBestDetection();

    default Optional<AprilTagDetection> findDetection(int fiducialId) {
        Optional<AprilTagResults> results = getResults();

        if (results.isPresent()) {
            List<AprilTagDetection> detections = results.get().results();

            for (AprilTagDetection detection : detections) {
                if (detection.getFiducialID() == fiducialId) {
                    return Optional.of(detection);
                }
            }
        }

        return Optional.empty();
    }

    default List<AprilTagDetection> findDetections(int... ids) {
        Optional<AprilTagResults> optResults = getResults();

        if (optResults.isEmpty()) return Collections.emptyList();

        AprilTagResults results = optResults.get();
        List<AprilTagDetection> detections = new ArrayList<>(results.results().size());

        for (AprilTagDetection tag : results.results()) {
            for (int id : ids) {
                if (id == tag.getFiducialID()) {
                    detections.add(tag);
                    break;
                }
            }
        }

        return detections;
    }
}
