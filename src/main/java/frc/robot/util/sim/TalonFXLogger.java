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
        backend.log("Motor Voltage", talon.getMotorVoltage().getValueAsDouble());
        backend.log("Supply Voltage", talon.getSupplyVoltage().getValueAsDouble());
        backend.log("Supply Current", talon.getSupplyCurrent().getValueAsDouble());
        backend.log("Torque Current", talon.getTorqueCurrent().getValueAsDouble());
        backend.log("Applied Rotor Velocity", talon.getAppliedRotorPolarity().getValueAsDouble());
        backend.log("Duty Cycle", talon.getDutyCycle().getValueAsDouble());

        backend.log("Tuning/Position", talon.getPosition().getValueAsDouble());
        backend.log("Tuning/Velocity", talon.getVelocity().getValueAsDouble());
        backend.log("Tuning/Acceleration", talon.getAcceleration().getValueAsDouble());
        backend.log("Tuning/Target Error", talon.getClosedLoopError().getValueAsDouble());
        backend.log("Tuning/Target Position", talon.getClosedLoopReference().getValueAsDouble());
        backend.log(
            "Tuning/Target Velocity", talon.getClosedLoopReferenceSlope().getValueAsDouble());
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
        backend.log("Active Closed Loop Slot", talon.getClosedLoopSlot().getValue());

        backend.log("Device Temp", talon.getDeviceTemp().getValueAsDouble());
        backend.log("Device ID", talon.getDeviceID());
        backend.log("Is Pro Licensed", talon.getIsProLicensed().getValue());

        backend.log("Faults/ProcTemp", talon.getFault_ProcTemp().getValue());
        backend.log("Faults/DeviceTemp", talon.getFault_DeviceTemp().getValue());
        backend.log("Faults/Undervoltage", talon.getFault_Undervoltage().getValue());
        backend.log("Faults/BootDuringEnable", talon.getFault_BootDuringEnable().getValue());
        backend.log("Faults/UnlicensedFeatureInUse", talon.getFault_UnlicensedFeatureInUse().getValue());
        backend.log("Faults/BridgeBrownout", talon.getFault_BridgeBrownout().getValue());
        backend.log("Faults/RemoteSensorPosOverflow", talon.getFault_RemoteSensorPosOverflow().getValue());
        backend.log("Faults/OverSupplyV", talon.getFault_OverSupplyV().getValue());
        backend.log("Faults/UnstableSupplyV", talon.getFault_UnstableSupplyV().getValue());
        backend.log("Faults/ReverseSoftLimit", talon.getFault_ReverseSoftLimit().getValue());
        backend.log("Faults/ForwardSoftLimit", talon.getFault_ForwardSoftLimit().getValue());
        backend.log("Faults/RemoteSensorDataInvalid", talon.getFault_RemoteSensorDataInvalid().getValue());
        backend.log("Faults/FusedSensorOutOfSync", talon.getFault_FusedSensorOutOfSync().getValue());
        backend.log("Faults/StatorCurrLimit", talon.getFault_StatorCurrLimit().getValue());
        backend.log("Faults/SupplyCurrLimit", talon.getFault_SupplyCurrLimit().getValue());
        backend.log("Faults/StaticBrakeDisabled", talon.getFault_StaticBrakeDisabled().getValue());

        backend.log("Sticky Faults/ProcTemp", talon.getStickyFault_ProcTemp().getValue());
        backend.log("Sticky Faults/DeviceTemp", talon.getStickyFault_DeviceTemp().getValue());
        backend.log("Sticky Faults/Undervoltage", talon.getStickyFault_Undervoltage().getValue());
        backend.log("Sticky Faults/BootDuringEnable", talon.getStickyFault_BootDuringEnable().getValue());
        backend.log("Sticky Faults/UnlicensedFeatureInUse", talon.getStickyFault_UnlicensedFeatureInUse().getValue());
        backend.log("Sticky Faults/BridgeBrownout", talon.getStickyFault_BridgeBrownout().getValue());
        backend.log("Sticky Faults/RemoteSensorPosOverflow", talon.getStickyFault_RemoteSensorPosOverflow().getValue());
        backend.log("Sticky Faults/OverSupplyV", talon.getStickyFault_OverSupplyV().getValue());
        backend.log("Sticky Faults/UnstableSupplyV", talon.getStickyFault_UnstableSupplyV().getValue());
        backend.log("Sticky Faults/ReverseSoftLimit", talon.getStickyFault_ReverseSoftLimit().getValue());
        backend.log("Sticky Faults/ForwardSoftLimit", talon.getStickyFault_ForwardSoftLimit().getValue());
        backend.log("Sticky Faults/RemoteSensorDataInvalid", talon.getStickyFault_RemoteSensorDataInvalid().getValue());
        backend.log("Sticky Faults/FusedSensorOutOfSync", talon.getStickyFault_FusedSensorOutOfSync().getValue());
        backend.log("Sticky Faults/StatorCurrLimit", talon.getStickyFault_StatorCurrLimit().getValue());
        backend.log("Sticky Faults/SupplyCurrLimit", talon.getStickyFault_SupplyCurrLimit().getValue());
        backend.log("Sticky Faults/StaticBrakeDisabled", talon.getStickyFault_StaticBrakeDisabled().getValue());
    }
}
