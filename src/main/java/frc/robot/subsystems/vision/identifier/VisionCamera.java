package frc.robot.subsystems.vision.identifier;

public enum VisionCamera {
    RADIO_CAM(VisionType.PHOTONVISION),
    SCORE_CAM(VisionType.PHOTONVISION),
    BELUGA_LIMELIGHT(VisionType.LIMELIGHT_MEGATAG_2);

    private final VisionType visionType;

    VisionCamera(VisionType visionType) {
        this.visionType = visionType;
    }

    public VisionType getVisionType() {
        return visionType;
    }
}
