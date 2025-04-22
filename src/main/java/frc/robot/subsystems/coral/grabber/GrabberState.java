package frc.robot.subsystems.coral.grabber;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.util.state.SetpointEnum;

public enum GrabberState implements SetpointEnum<Dimensionless, DimensionlessUnit> {
    ROLL_OUT(-0.8),
    ROLL_IN(1),
    OFF(0);

    private final Dimensionless speed;

    GrabberState(double speed) {
        this.speed = Units.Value.of(speed);
    }

    @Override
    public Dimensionless getSetpoint() {
        return speed;
    }

    @Override
    public Dimensionless getTolerance() {
        return Units.Value.zero();
    }
}
