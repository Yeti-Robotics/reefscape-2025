package frc.robot.util.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class CoralManipulatorMechanismVisualizer {
    private final LoggedMechanism2d elevatorArmMech =
            new LoggedMechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d liftLigament;
    private static final CoralManipulatorMechanismVisualizer visualizerInstance = new CoralManipulatorMechanismVisualizer();

    private CoralManipulatorMechanismVisualizer() {
        liftLigament =
                elevatorArmMech
                        .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                        .append(
                                new LoggedMechanismLigament2d(
                                        "lift",
                                        Units.feetToMeters(3),
                                        90,
                                        6,
                                        new Color8Bit(Color.kRed)));
        elevatorArmMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(
                        new LoggedMechanismLigament2d(
                                "bottom",
                                Units.feetToMeters(3),
                                0,
                                6,
                                new Color8Bit(Color.kGreen)));
        armLigament =
                liftLigament.append(
                        new LoggedMechanismLigament2d(
                                "arm", Units.inchesToMeters(12), 0, 6, new Color8Bit(Color.kBlue)));
    }

    public static CoralManipulatorMechanismVisualizer getInstance() {
        return visualizerInstance;
    }

    public LoggedMechanismLigament2d getArmLigament() {
        return armLigament;
    }

    public LoggedMechanismLigament2d getLiftLigament() {
        return liftLigament;
    }

    @AutoLogOutput(key = "Mechanism/CoralManipulatorVisualizer")
    public LoggedMechanism2d getElevatorArmMech() {
        return elevatorArmMech;
    }
}
