package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.sim.PhysicsSim;

public class ArmIOSimulatedTalonFX implements ArmIO {
    private final ArmIOTalonFX talonFXIO = new ArmIOTalonFX();

    public ArmIOSimulatedTalonFX() {
        PhysicsSim.getInstance().addTalonFX(talonFXIO.armMotor, talonFXIO.armCancoder);
    }

    @Override
    public Angle getPosition() {
        return talonFXIO.getPosition();
    }

    @Override
    public void setPosition(Angle value) {
        talonFXIO.setPosition(value);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint) {
        return talonFXIO.isAtSetpoint(setpoint);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint, Angle tolerance) {
        return talonFXIO.isAtSetpoint(setpoint, tolerance);
    }
}
