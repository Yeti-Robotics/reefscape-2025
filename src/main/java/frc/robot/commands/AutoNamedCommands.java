package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coral.*;

public class AutoNamedCommands {
    private final CoralManipulatorSystem coralManipulator;

    public AutoNamedCommands(CoralManipulatorSystem coralManipulator) {
        this.coralManipulator = coralManipulator;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "GroundIntake", coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        NamedCommands.registerCommand(
                "HPIntake", coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));

        NamedCommands.registerCommand(
                "L1",
                new SequentialCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.L1),
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L1)));
        NamedCommands.registerCommand(
                "L2",
                new SequentialCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.L2),
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L2)));
        NamedCommands.registerCommand(
                "L3",
                new SequentialCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.L3),
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3)));
        NamedCommands.registerCommand(
                "L4",
                new SequentialCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.L4),
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4)));
    }
}
