package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.sim.PhysicsSim;

public class ArmStateSetpointIOSimulatedTalonFX implements ArmStateSetpointIO {
    private final ArmStateSetpointIOTalonFX talonFXIO = new ArmStateSetpointIOTalonFX();

    public ArmStateSetpointIOSimulatedTalonFX() {
        PhysicsSim.getInstance().addTalonFX(talonFXIO.armMotor, talonFXIO.armCancoder);
    }

    @Override
    public Angle getState() {
        return talonFXIO.getState();
    }

    @Override
    public void setState(Angle value) {
        talonFXIO.setState(value);
    }
}
