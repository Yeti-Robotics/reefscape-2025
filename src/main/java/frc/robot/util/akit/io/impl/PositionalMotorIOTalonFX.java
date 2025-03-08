package frc.robot.util.akit.io.impl;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.io.PositionalMotorIO;

public abstract class PositionalMotorIOTalonFX<>
        implements PositionalMotorIO<Angle> {
    protected final TalonFX talon;
    private final StatusSignal<Angle> positionSignal;

    public PositionalMotorIOTalonFX(TalonFX talon) {
        this.talon = talon;
        positionSignal = talon.getPosition();
    }



    @Override
    public Angle getPosition() {
        positionSignal.refresh();
        return positionSignal.getValue();
    }
}
