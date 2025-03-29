package frc.robot.commands;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.coral.grabber.GrabberState;

public class AutoNamedCommands {
    private final CoralManipulatorSystem coralManipulator;
    private final ReefAlignCommand reefAlignCommand;

    public AutoNamedCommands(
            CoralManipulatorSystem coralManipulator, ReefAlignCommand reefAlignCommand) {
        this.coralManipulator = coralManipulator;
        this.reefAlignCommand = reefAlignCommand;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "GroundIntake", coralManipulator.transitionTo(CoralManipulatorState.GROUND_INTAKE));
        NamedCommands.registerCommand(
                "HPIntake", coralManipulator.transitionTo(CoralManipulatorState.HP_INTAKE));
        NamedCommands.registerCommand(
                "LollipopIntake", coralManipulator.transitionTo(CoralManipulatorState.LOLLIPOP));

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
                "Score1",coralManipulator.transitionTo(CoralManipulatorState.SCORE_L1));
        NamedCommands.registerCommand(
                "Score2",coralManipulator.transitionTo(CoralManipulatorState.SCORE_L2));
        NamedCommands.registerCommand(
                "Score3",coralManipulator.transitionTo(CoralManipulatorState.SCORE_L3));
        NamedCommands.registerCommand(
                "Score4",coralManipulator.transitionTo(CoralManipulatorState.SCORE_L4));

        NamedCommands.registerCommand(
                "Stow", coralManipulator.transitionTo(CoralManipulatorState.STOWED));
    }
}
