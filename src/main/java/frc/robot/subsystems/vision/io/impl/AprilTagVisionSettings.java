package frc.robot.subsystems.vision.io.impl;

import java.util.EnumSet;

public class AprilTagVisionSettings {
    public enum VisionAprilTagOptions {
        LOCALIZATION,
        BEST_DETECTION,
        ALL_DETECTIONS,
    }

    private final EnumSet<VisionAprilTagOptions> visionAprilTagOptions = EnumSet.noneOf(VisionAprilTagOptions.class);
    private final VisionAprilTagMode settings = new VisionAprilTagMode(visionAprilTagOptions);

    public static class VisionAprilTagMode {
        private final EnumSet<VisionAprilTagOptions> visionAprilTagOptions;

        public VisionAprilTagMode(EnumSet<VisionAprilTagOptions> visionAprilTagOptions) {
            this.visionAprilTagOptions = visionAprilTagOptions;
        }

        public boolean hasDisabledAll() {
            return visionAprilTagOptions.isEmpty();
        }

        public boolean hasEnabled(VisionAprilTagOptions... mode) {
            for (VisionAprilTagOptions m : mode) {
                if (!visionAprilTagOptions.contains(m)) {
                    return false;
                }
            }

            return true;
        }
    }

    public static AprilTagVisionSettings defaultMode() {
        return new AprilTagVisionSettings()
                .enable(VisionAprilTagOptions.LOCALIZATION, VisionAprilTagOptions.BEST_DETECTION);
    }

    @SuppressWarnings({"UseBulkOperation", "ManualArrayToCollectionCopy"})
    public AprilTagVisionSettings enable(VisionAprilTagOptions... mode) {
        for (VisionAprilTagOptions m : mode) {
            visionAprilTagOptions.add(m);
        }

        return this;
    }

    public AprilTagVisionSettings disable(VisionAprilTagOptions... mode) {
        for (VisionAprilTagOptions m : mode) {
            visionAprilTagOptions.remove(m);
        }

        return this;
    }

    public AprilTagVisionSettings enableAll() {
        visionAprilTagOptions.addAll(EnumSet.allOf(VisionAprilTagOptions.class));
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

    public VisionAprilTagMode settings() {
        return settings;
    }
}
