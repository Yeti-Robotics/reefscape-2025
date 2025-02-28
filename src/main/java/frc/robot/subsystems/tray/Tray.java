package frc.robot.subsystems.tray;

import com.ctre.phoenix6.Utils;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.util.StateManager;

public class Tray extends SubsystemBase {
    public final Canandcolor traySensor = new Canandcolor(0);

    public final Trigger coralInTrayTrigger;
    public final Trigger algaeInTrayTrigger;

    public enum TrayState {
        ALGAE,
        CORAL,
        EMPTY
    }

    boolean isInTray = false;

    // public StateManager<TrayState> trayState = new StateManager<>(TrayState.EMPTY);

    public Tray() {
        coralInTrayTrigger = new Trigger(this::isCoralInTray);
        algaeInTrayTrigger = new Trigger(this::isAlgaeInTray);
        SmartDashboard.putData(new InstantCommand(() -> isInTray = !isInTray));
    }

    // @Override
    //    public void periodic() {
    //        switch (trayState.getState()) {
    //            case ALGAE:
    //                break;
    //            case CORAL:
    //                break;
    //            case EMPTY:
    //                break;
    //        }
    //    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IsCoralInTray", isCoralInTray());
    }

    public boolean isCoralInTray() {
        ColorData traySensorColor = traySensor.getColor();

        return Utils.isSimulation()
                ? isInTray
                : traySensorColor.hue() >= 0.7 && traySensor.getProximity() < 0.1;

        /*
        return traySensorColor.red() == TrayConfigs.coralColor.red()
                && traySensorColor.green() == TrayConfigs.coralColor.green()
                && traySensorColor.blue() == TrayConfigs.coralColor.blue();

         */
    }

    public boolean isAlgaeInTray() {
        ColorData traySensorColor = traySensor.getColor();

        return traySensorColor.hue() >= 0.4
                && traySensorColor.hue() < 0.7
                && traySensor.getProximity() < 0.1;

        /*
        return traySensorColor.red() == TrayConfigs.algaeColor.red()
                && traySensorColor.green() == TrayConfigs.algaeColor.green()
                && traySensorColor.blue() == TrayConfigs.algaeColor.blue();

         */
    }
}
