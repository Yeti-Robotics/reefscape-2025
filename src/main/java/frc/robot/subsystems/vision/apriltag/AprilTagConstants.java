package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.io.IOException;

public class AprilTagConstants {
    private static final String TAGS_FILE = "apriltags.json";
    public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(TAGS_FILE);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
