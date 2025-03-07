package frc.robot.util.akit.logging.inputs.impl;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.akit.io.InputLoggingIO;

public class CurrentInputs {
    public double supplyVoltage;
    public double supplyAmps;
    public double statorAmps;
    public double torqueAmps;
    public double stallAmps;

    public static class CurrentInputsTalonFX implements InputLoggingIO<CurrentInputs> {
        private final StatusSignal<Voltage> supplyVoltageSignal;
        private final StatusSignal<Current> supplyAmpsSignal;
        private final StatusSignal<Current> statorAmpsSignal;
        private final StatusSignal<Current> torqueAmpsSignal;
        private final StatusSignal<Current> stallAmpsSignal;

        private final StatusSignal<?>[] statusSignals;

        public CurrentInputsTalonFX(TalonFX talon) {
            supplyVoltageSignal = talon.getSupplyVoltage();
            supplyAmpsSignal = talon.getSupplyCurrent();
            statorAmpsSignal = talon.getSupplyCurrent();
            torqueAmpsSignal = talon.getTorqueCurrent();
            stallAmpsSignal = talon.getMotorStallCurrent();

            statusSignals =
                    new StatusSignal[] {
                        supplyVoltageSignal,
                        supplyAmpsSignal,
                        statorAmpsSignal,
                        torqueAmpsSignal,
                        stallAmpsSignal
                    };
        }

        public StatusSignal<?>[] statusSignals() {
            return statusSignals;
        }

        @Override
        public void updateInputs(CurrentInputs inputs) {
            BaseStatusSignal.refreshAll(statusSignals);

            inputs.stallAmps = stallAmpsSignal.getValueAsDouble();
            inputs.torqueAmps = torqueAmpsSignal.getValueAsDouble();
            inputs.statorAmps = statorAmpsSignal.getValueAsDouble();
            inputs.supplyAmps = supplyAmpsSignal.getValueAsDouble();
            inputs.supplyVoltage = supplyVoltageSignal.getValueAsDouble();
        }
    }
}
