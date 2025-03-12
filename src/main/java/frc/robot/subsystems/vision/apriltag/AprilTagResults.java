package frc.robot.subsystems.vision.apriltag;

import java.util.Collections;
import java.util.List;

public class AprilTagResults {
    private final List<AprilTagDetection> results;
    private final double timestamp;
    private final double latency;

    public AprilTagResults(double timestamp, double latency, List<AprilTagDetection> results) {
        this.results = Collections.unmodifiableList(results);
        this.timestamp = timestamp;
        this.latency = latency;
    }

    public List<AprilTagDetection> getResults() {
        return results;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getLatency() {
        return latency;
    }
}
