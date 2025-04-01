package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.coral.grabber.GrabberState;

public class AutoNamedCommands {
    private final CoralManipulatorSystem coralManipulator;
    private final Command reefAlignCommand;

    public AutoNamedCommands(CoralManipulatorSystem coralManipulator, Command reefAlignCommand) {
        this.coralManipulator = coralManipulator;
        this.reefAlignCommand = reefAlignCommand;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "GroundIntake", coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        NamedCommands.registerCommand(
                "HPIntake", coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));

        NamedCommands.registerCommand("Reef align", reefAlignCommand);

        NamedCommands.registerCommand(
                "L1", coralManipulator.transitionTo(CoralManipulatorState.L1));
        NamedCommands.registerCommand(
                "L2", coralManipulator.transitionTo(CoralManipulatorState.L2));
        NamedCommands.registerCommand(
                "L3", coralManipulator.transitionTo(CoralManipulatorState.L3));
        NamedCommands.registerCommand(
                "L4", coralManipulator.transitionTo(CoralManipulatorState.L4));

        NamedCommands.registerCommand(
                "Rollout", coralManipulator.grabber.transitionTo(GrabberState.ROLL_OUT));

        NamedCommands.registerCommand(
                "Score1",
                new ParallelCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L1),
                        new WaitCommand(1)));
        NamedCommands.registerCommand(
                "Score2",
                new ParallelCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L2),
                        new WaitCommand(1)));
        NamedCommands.registerCommand(
                "Score3",
                new ParallelCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3),
                        new WaitCommand(1)));
        NamedCommands.registerCommand(
                "Score4",
                new ParallelCommandGroup(
                        coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4),
                        new WaitCommand(1)));

        NamedCommands.registerCommand(
                "Stow", coralManipulator.transitionTo(CoralManipulatorState.STOWED));
    }
}
