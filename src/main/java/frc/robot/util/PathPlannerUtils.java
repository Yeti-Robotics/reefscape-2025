package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import java.util.Optional;

public class PathPlannerUtils {
    public static Optional<PathPlannerPath> loadPathByName(String name) {
        try {
            return Optional.of(PathPlannerPath.fromPathFile(name));
        } catch (Exception e) {
            System.err.println("PathPlannerPath " + name + " failed to load");
            return Optional.empty();
        }
    }
}
