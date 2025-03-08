package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform2d;

import javax.xml.crypto.dsig.Transform;

public class FieldConstants {
    public static final double FIELD_LENGTH = 16.5354;
    public static final Transform2d RIGHT_BRANCH_TRANSFORM = new Transform2d(new Translation2d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(6.482)),
            new Rotation2d(0, 0));
    public static final Transform2d LEFT_BRANCH_TRANSFORM = new Transform2d(new Translation2d(
            Units.inchesToMeters(-12),
            Units.inchesToMeters(-6.482)),
            new Rotation2d(0, 0));
}
