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
        backend.log("getTorqueCurrent", talon.getTorqueCurrent().getValueAsDouble());
        backend.log("getAppliedRotorPolarity", talon.getAppliedRotorPolarity().getValueAsDouble());

        backend.log("getPosition", talon.getPosition().getValueAsDouble());
        backend.log("getVelocity", talon.getVelocity().getValueAsDouble());
        backend.log("getAcceleration", talon.getAcceleration().getValueAsDouble());
        backend.log("getClosedLoopError", talon.getClosedLoopError().getValueAsDouble());
        backend.log("getClosedLoopReference", talon.getClosedLoopReference().getValueAsDouble());
        backend.log(
                "getClosedLoopReferenceSlope",
                talon.getClosedLoopReferenceSlope().getValueAsDouble());
        backend.log("getClosedLoopOutput", talon.getClosedLoopOutput().getValueAsDouble());
        backend.log(
                "getClosedLoopProportionalOutput",
                talon.getClosedLoopProportionalOutput().getValueAsDouble());
        backend.log(
                "getClosedLoopIntegratedOutput",
                talon.getClosedLoopIntegratedOutput().getValueAsDouble());
        backend.log(
                "getClosedLoopDerivativeOutput",
                talon.getClosedLoopDerivativeOutput().getValueAsDouble());
        backend.log("getAppliedControl", talon.getAppliedControl().getControlInfo().toString());
        backend.log("getForwardLimit", talon.getForwardLimit().getValueAsDouble());
        backend.log("getReverseLimit", talon.getReverseLimit().getValueAsDouble());

        backend.log("getDeviceTemp", talon.getDeviceTemp().getValueAsDouble());
        backend.log("getDeviceID", talon.getDeviceID());
        backend.log("getDutyCycle", talon.getDutyCycle().getValueAsDouble());
    }
}
