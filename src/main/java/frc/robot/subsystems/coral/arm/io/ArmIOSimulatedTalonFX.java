package frc.robot.subsystems.coral.arm.io;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.akit.io.PeriodicIO;
import frc.robot.util.sim.CoralManipulatorMechanismVisualizer;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class ArmIOSimulatedTalonFX
        implements ArmIO, PeriodicIO {

    private final ArmIOTalonFX talonFXIO = new ArmIOTalonFX();

    public ArmIOSimulatedTalonFX() {
        PhysicsSim.getInstance().addTalonFX(talonFXIO.armMotor, talonFXIO.armCancoder);
    }


    @Override
    public void updateInputs(ArmInputs inputs) {
        talonFXIO.updateInputs(inputs);
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
    public void periodic() {
        double simulatedArmAngle = (talonFXIO.getPosition().in(Units.Rotations) * 360.0) - 90;
        CoralManipulatorMechanismVisualizer.getInstance().getArmLigament().setAngle(simulatedArmAngle);
    }
}
