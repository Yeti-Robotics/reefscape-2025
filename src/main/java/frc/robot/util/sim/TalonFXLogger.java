package frc.robot.util.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    protected void update(EpilogueBackend backend, TalonFX talon) {
        backend.log("Supply Voltage", talon.getSupplyVoltage().getValueAsDouble());
        backend.log("Supply Current", talon.getSupplyCurrent().getValueAsDouble());
        backend.log("Torque Current", talon.getTorqueCurrent().getValueAsDouble());
        backend.log("Applied Rotor Velocity", talon.getAppliedRotorPolarity().getValueAsDouble());
        backend.log("Duty Cycle", talon.getDutyCycle().getValueAsDouble());

        backend.log("Position", talon.getPosition().getValueAsDouble());
        backend.log("Velocity", talon.getVelocity().getValueAsDouble());
        backend.log("Acceleration", talon.getAcceleration().getValueAsDouble());
        backend.log("Target Error", talon.getClosedLoopError().getValueAsDouble());
        backend.log("Target Position", talon.getClosedLoopReference().getValueAsDouble());
        backend.log("Target Velocity", talon.getClosedLoopReferenceSlope().getValueAsDouble());
        backend.log("Closed Loop Output", talon.getClosedLoopOutput().getValueAsDouble());
        backend.log(
                "Closed Loop P Output", talon.getClosedLoopProportionalOutput().getValueAsDouble());
        backend.log(
                "Closed Loop I Output", talon.getClosedLoopIntegratedOutput().getValueAsDouble());
        backend.log(
                "Closed Loop D Output", talon.getClosedLoopDerivativeOutput().getValueAsDouble());
        backend.log("Applied Control", talon.getAppliedControl().getControlInfo().toString());
        backend.log("Forward Limit", talon.getForwardLimit().getValueAsDouble());
        backend.log("Reverse Limit", talon.getReverseLimit().getValueAsDouble());

        backend.log("Device Temp", talon.getDeviceTemp().getValueAsDouble());
        backend.log("Device ID", talon.getDeviceID());
    }
}
