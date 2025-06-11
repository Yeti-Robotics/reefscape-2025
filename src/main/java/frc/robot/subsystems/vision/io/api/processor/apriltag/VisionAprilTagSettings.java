package frc.robot.subsystems.vision.io.api.processor.apriltag;

import java.util.Collections;
import java.util.EnumSet;

public final class VisionAprilTagSettings {
    private final EnumSet<VisionAprilTagFeature> features;

    public enum VisionAprilTagFeature {
        LOCALIZATION,
        BEST_DETECTION,
        ALL_DETECTIONS
    }

    private VisionAprilTagSettings(VisionAprilTagFeature... features) {
        this.features = EnumSet.of(features[0], features);
    }

    public static VisionAprilTagSettings defaultSettings() {
        return new VisionAprilTagSettings(VisionAprilTagFeature.BEST_DETECTION, VisionAprilTagFeature.LOCALIZATION);
    }

    public static VisionAprilTagSettings of(VisionAprilTagFeature... features) {
        return new VisionAprilTagSettings(features);
    }

    public VisionAprilTagSettings enable(VisionAprilTagFeature... toEnable) {
        Collections.addAll(features, toEnable);
        return this;
    }

    public VisionAprilTagSettings disable(VisionAprilTagFeature... toDisable) {
        for (VisionAprilTagFeature f : toDisable) features.remove(f);
        return this;
    }

    public VisionAprilTagSettings set(VisionAprilTagFeature... toSet) {
        features.clear();
        return enable(toSet);
    }

    public boolean hasEnabledAll(VisionAprilTagFeature... check) {
        for (VisionAprilTagFeature f : check) {
            if (!features.contains(f)) return false;
        }
        return true;
    }

    public boolean hasEnabledAny(VisionAprilTagFeature... check) {
        for (VisionAprilTagFeature f : check) {
            if (features.contains(f)) return true;
        }
        return false;
    }

    public boolean hasDisabledAll() {
        return features.isEmpty();
    }

    @Override
    public String toString() {
        return "VisionAprilTagSettings" + features;
    }
}


