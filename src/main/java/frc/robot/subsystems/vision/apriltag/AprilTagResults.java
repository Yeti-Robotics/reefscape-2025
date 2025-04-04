package frc.robot.subsystems.vision.apriltag;

import java.util.Collections;
import java.util.List;

public record AprilTagResults(double timestamp, double latency, List<AprilTagDetection> results) {
    public AprilTagResults(double timestamp, double latency, List<AprilTagDetection> results) {
        this.timestamp = timestamp;
        this.latency = latency;
        this.results = Collections.unmodifiableList(results);
    }
}
