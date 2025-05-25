package frc.robot.subsystems.vision.io.api;

import java.util.Collections;
import java.util.EnumSet;
import java.util.Set;

public final class VisionAprilTagSettings {
    public enum VisionAprilTagFeature {
        LOCALIZATION,
        BEST_DETECTION,
        ALL_DETECTIONS,
    }

    public static final VisionAprilTagSettings DEFAULT = new VisionAprilTagSettings(EnumSet.of(
            VisionAprilTagFeature.LOCALIZATION,
            VisionAprilTagFeature.BEST_DETECTION
    ));

    public static final VisionAprilTagSettings OFF = new VisionAprilTagSettings(EnumSet.noneOf(VisionAprilTagFeature.class));

    private final Set<VisionAprilTagFeature> features;

    private VisionAprilTagSettings(Set<VisionAprilTagFeature> features) {
        // Defensive copy and unmodifiable
        this.features = Collections.unmodifiableSet(EnumSet.copyOf(features));
    }

    public VisionAprilTagSettings enable(VisionAprilTagFeature... newFeatures) {
        EnumSet<VisionAprilTagFeature> updated = EnumSet.copyOf(features);
        Collections.addAll(updated, newFeatures);

        return from(updated);
    }

    public VisionAprilTagSettings disable(VisionAprilTagFeature... removeFeatures) {
        EnumSet<VisionAprilTagFeature> updated = EnumSet.copyOf(features);
        for (VisionAprilTagFeature f : removeFeatures) {
            updated.remove(f);
        }

        return from(updated);
    }

    public boolean hasEnabledAll(VisionAprilTagFeature... modes) {
        for (VisionAprilTagFeature m : modes) {
            if (!features.contains(m)) {
                return false;
            }
        }
        return true;
    }

    public boolean hasEnabledAny(VisionAprilTagFeature... modes) {
        for (VisionAprilTagFeature m : modes) {
            if (features.contains(m)) {
                return true;
            }
        }
        return false;
    }

    public boolean hasDisabledAll() {
        return features.isEmpty();
    }

    private static VisionAprilTagSettings from(Set<VisionAprilTagFeature> candidate) {
        if (candidate.equals(DEFAULT.features)) {
            return DEFAULT;
        } else if (candidate.isEmpty()) {
            return OFF;
        } else {
            return new VisionAprilTagSettings(candidate);
        }
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof VisionAprilTagSettings that)) return false;
        return features.equals(that.features);
    }

    @Override
    public String toString() {
        return "VisionAprilTagSettings" + features;
    }
}
