package frc.robot.subsystems.vision.data.apriltag;

import java.util.NavigableMap;
import java.util.TreeMap;

public class VisionAprilTagObserver {
    private final NavigableMap<Double, VisionAprilTag3D> timestampedTags = new TreeMap<>();
    private final int trackedID;

    public VisionAprilTagObserver(int trackedID) {
        this.trackedID = trackedID;
    }

    public void add(VisionAprilTag3D tag) {
        VisionAprilTag3D copyTag = tag.copy();
        timestampedTags.put(copyTag.timestamp(), copyTag);
    }

    public VisionAprilTag3D getBestTag() {
        return timestampedTags.lastEntry().getValue();
    }
}
