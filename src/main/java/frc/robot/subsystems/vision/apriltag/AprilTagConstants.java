package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.IOException;

public class AprilTagConstants {
    //    private static final String TAGS_FILE = "apriltags.json";
    public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            APRIL_TAG_FIELD_LAYOUT =
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        }
    }
}
