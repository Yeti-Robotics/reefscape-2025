package frc.robot.subsystems.coral;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import static edu.wpi.first.units.Units.Degrees;

public class CoralManipulatorMechanismVisualizer {
    private static final String KEY_PREFIX = "Mechanisms/CoralManipulator/";
    private final LoggedMechanism2d elevatorArmMech =
            new LoggedMechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d elevatorLigament;
    private final String key;

    protected CoralManipulatorMechanismVisualizer(String key) {
        elevatorLigament = elevatorArmMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new LoggedMechanismLigament2d("lift", Units.feetToMeters(3), 90, 6, new Color8Bit(Color.kRed)));
        elevatorArmMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new LoggedMechanismLigament2d(
                        "bottom", Units.feetToMeters(3), 0, 6, new Color8Bit(Color.kGreen)));
        armLigament = elevatorLigament.append(
                new LoggedMechanismLigament2d("arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));

        this.key = KEY_PREFIX + key;
    }

    public double scaleLength(double elevatorPosition) {
        return Units.inchesToMeters((elevatorPosition * 6) + 1);
    }

    public void update(Angle armPosition, Angle elevatorPosition) {
        armLigament.setAngle(armPosition.in(Degrees) - 90.0);
        elevatorLigament.setLength(scaleLength(elevatorPosition.magnitude()));

        Logger.recordOutput(key, elevatorArmMech);
    }
}
