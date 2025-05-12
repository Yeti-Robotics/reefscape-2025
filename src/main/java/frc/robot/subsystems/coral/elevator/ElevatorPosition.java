package frc.robot.subsystems.coral.elevator;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.coral.elevator.io.ElevatorConfig;
import frc.robot.util.state.MeasuredSetpointProvider;

public enum ElevatorPosition implements MeasuredSetpointProvider<Angle, AngleUnit> {
    BOTTOM(0.0),
    SAFE_POSITION(2.2),
    POS_L1(2.4),
    POS_L2(0.4),
    POS_L3(2.2),
    SCORE_L3(1.8),
    POS_L4(3.95),
    SCORE_L4(3.55), // 3.4
    HOLD(-1); // special case, for when transitions are interrupted

    private final Angle height;

    ElevatorPosition(double height) {
        this(Units.Rotations.of(height));
    }

    ElevatorPosition(Angle height) {
        this.height = height;
    }

    @Override
    public Angle getSetpoint() {
        return height;
    }

    @Override
    public Angle getTolerance() {
        return ElevatorConfig.HEIGHT_TOLERANCE;
    }
}
