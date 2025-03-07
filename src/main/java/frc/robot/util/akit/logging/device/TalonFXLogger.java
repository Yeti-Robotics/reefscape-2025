package frc.robot.util.akit.logging.device;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.List;
import org.littletonrobotics.junction.LogTable;

public class TalonFXLogger extends DeviceLogger {
    private final StatusSignal<?>[] statusSignals;

    public TalonFXLogger(List<StatusSignal<?>> statusSignals) {
        this.statusSignals = statusSignals.toArray(StatusSignal[]::new);
    }

    @Override
    public void toLog(LogTable table) {
        BaseStatusSignal.refreshAll(statusSignals);

        super.toLog(table);
    }
}
