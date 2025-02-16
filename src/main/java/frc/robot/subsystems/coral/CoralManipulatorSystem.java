package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.arm.ArmSubsystem;
import frc.robot.subsystems.coral.elevator.ElevatorSubsystem;
import frc.robot.subsystems.coral.grabber.GrabberSubsystem;

public class CoralManipulatorSystem extends SubsystemBase {
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final GrabberSubsystem grabber = new GrabberSubsystem();


}
