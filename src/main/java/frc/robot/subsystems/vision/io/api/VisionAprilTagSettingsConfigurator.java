package frc.robot.subsystems.vision.io.api;

import java.util.EnumSet;

public class VisionAprilTagSettingsConfigurator {
    public enum VisionAprilTagFeature {
        LOCALIZATION,
        BEST_DETECTION,
        ALL_DETECTIONS,
    }

    private final EnumSet<VisionAprilTagFeature> visionAprilTagOptions = EnumSet.noneOf(VisionAprilTagFeature.class);
    private final VisionAprilTagSettings settings = new VisionAprilTagSettings(visionAprilTagOptions);

    public static class VisionAprilTagSettings {
        private final EnumSet<VisionAprilTagFeature> visionAprilTagOptions;

        public VisionAprilTagSettings(EnumSet<VisionAprilTagFeature> visionAprilTagOptions) {
            this.visionAprilTagOptions = visionAprilTagOptions;
        }

        public boolean hasDisabledAll() {
            return visionAprilTagOptions.isEmpty();
        }

        public boolean hasEnabledAll(VisionAprilTagFeature... mode) {
            for (VisionAprilTagFeature m : mode) {
                if (!visionAprilTagOptions.contains(m)) {
                    return false;
                }
            }

            return true;
        }

        public boolean hasEnabledAny(VisionAprilTagFeature... mode) {
            for (VisionAprilTagFeature m : mode) {
                if (visionAprilTagOptions.contains(m)) {
                    return true;
                }
            }

            return false;
        }
    }

    private VisionAprilTagSettingsConfigurator() {}

    public static VisionAprilTagSettingsConfigurator defaultSettingsConfig() {
        return new VisionAprilTagSettingsConfigurator()
                .enable(VisionAprilTagFeature.LOCALIZATION, VisionAprilTagFeature.BEST_DETECTION);
    }

    public static VisionAprilTagSettingsConfigurator blankSettingsConfig() {
        return new VisionAprilTagSettingsConfigurator();
    }

    @SuppressWarnings({"UseBulkOperation", "ManualArrayToCollectionCopy"})
    public VisionAprilTagSettingsConfigurator enable(VisionAprilTagFeature... mode) {
        for (VisionAprilTagFeature m : mode) {
            visionAprilTagOptions.add(m);
        }

        return this;
    }

    public VisionAprilTagSettingsConfigurator disable(VisionAprilTagFeature... mode) {
        for (VisionAprilTagFeature m : mode) {
            visionAprilTagOptions.remove(m);
        }

        return this;
    }

    public VisionAprilTagSettingsConfigurator enableAll() {
        visionAprilTagOptions.addAll(EnumSet.allOf(VisionAprilTagFeature.class));
        return this;
    }

    public VisionAprilTagSettingsConfigurator disableAll() {
        visionAprilTagOptions.clear();
        return this;
    }

    public VisionAprilTagSettingsConfigurator apply(VisionAprilTagSettingsConfigurator otherFeature) {
        visionAprilTagOptions.addAll(otherFeature.visionAprilTagOptions);
        return this;
    }

    public VisionAprilTagSettings toSettings() {
        return settings;
    }
}
