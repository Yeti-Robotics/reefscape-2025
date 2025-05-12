package frc.robot.subsystems.vision;

public enum VisionCameraID {
    RADIO_CAM("radio_cam", VisionType.PHOTONVISION),
    SCORE_CAM("score_cam", VisionType.PHOTONVISION),
    BELUGA_LIMELIGHT("limelight", VisionType.LIMELIGHT_MEGATAG_2);

    public enum VisionType {
        LIMELIGHT_MEGATAG_2,
        PHOTONVISION
    }

    public final String cameraName;
    public final VisionType visionType;

    VisionCameraID(String cameraName, VisionType visionType) {
        this.cameraName = cameraName;
        this.visionType = visionType;
    }

    /**
     * Gets the camera name.
     * 
     * @return The camera name
     */
    public String getCameraName() {
        return cameraName;
    }
}
