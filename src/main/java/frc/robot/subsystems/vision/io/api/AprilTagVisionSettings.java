package frc.robot.subsystems.vision.io.api;

import java.util.EnumSet;

public class AprilTagVisionSettings {
    public enum AprilTagVisionFeatures {
        LOCALIZATION,
        BEST_DETECTION,
        ALL_DETECTIONS,
    }

    private final EnumSet<AprilTagVisionFeatures> visionAprilTagOptions = EnumSet.noneOf(AprilTagVisionFeatures.class);
    private final AprilTagVisionMode settings = new AprilTagVisionMode(visionAprilTagOptions);

    public static class AprilTagVisionMode {
        private final EnumSet<AprilTagVisionFeatures> visionAprilTagOptions;

        public AprilTagVisionMode(EnumSet<AprilTagVisionFeatures> visionAprilTagOptions) {
            this.visionAprilTagOptions = visionAprilTagOptions;
        }

        public boolean hasDisabledAll() {
            return visionAprilTagOptions.isEmpty();
        }

        public boolean hasEnabled(AprilTagVisionFeatures... mode) {
            for (AprilTagVisionFeatures m : mode) {
                if (!visionAprilTagOptions.contains(m)) {
                    return false;
                }
            }

            return true;
        }
    }

    private AprilTagVisionSettings() {}

    public static AprilTagVisionSettings defaultSettings() {
        return new AprilTagVisionSettings()
                .enable(AprilTagVisionFeatures.LOCALIZATION, AprilTagVisionFeatures.BEST_DETECTION);
    }

    public static AprilTagVisionSettings blankSettings() {
        return new AprilTagVisionSettings();
    }

    @SuppressWarnings({"UseBulkOperation", "ManualArrayToCollectionCopy"})
    public AprilTagVisionSettings enable(AprilTagVisionFeatures... mode) {
        for (AprilTagVisionFeatures m : mode) {
            visionAprilTagOptions.add(m);
        }

        return this;
    }

    public AprilTagVisionSettings disable(AprilTagVisionFeatures... mode) {
        for (AprilTagVisionFeatures m : mode) {
            visionAprilTagOptions.remove(m);
        }

        return this;
    }

    public AprilTagVisionSettings enableAll() {
        visionAprilTagOptions.addAll(EnumSet.allOf(AprilTagVisionFeatures.class));
        return this;
    }

    public AprilTagVisionSettings disableAll() {
        visionAprilTagOptions.clear();
        return this;
    }

    public AprilTagVisionSettings apply(AprilTagVisionSettings otherFeature) {
        visionAprilTagOptions.addAll(otherFeature.visionAprilTagOptions);
        return this;
    }

    public AprilTagVisionMode settings() {
        return settings;
    }
}
