package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.coral.*;

public class AutoNamedCommands {
    private final CoralManipulatorSystem coralManipulator;

    public AutoNamedCommands(CoralManipulatorSystem coralManipulator) {
        this.coralManipulator = coralManipulator;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "intake", coralManipulator.transitionTo(CoralManipulatorState.INTAKE_CORAL));

        NamedCommands.registerCommand(
                "L1", coralManipulator.transitionTo(CoralManipulatorState.L1));
        NamedCommands.registerCommand(
                "L2", coralManipulator.transitionTo(CoralManipulatorState.L2));
        NamedCommands.registerCommand(
                "L3", coralManipulator.transitionTo(CoralManipulatorState.L3));
        NamedCommands.registerCommand(
                "L4", coralManipulator.transitionTo(CoralManipulatorState.L4));

        NamedCommands.registerCommand("scoreState", coralManipulator.scoreState());
    }
}
