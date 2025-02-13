package frc.robot.util.sim;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import java.util.List;

public interface Simulatable {
    record MechanismBundle(
            String name, Mechanism2d mechanism, List<MechanismLigament2d> ligaments) {}

    MechanismBundle buildMechanism();

    void update();
}
