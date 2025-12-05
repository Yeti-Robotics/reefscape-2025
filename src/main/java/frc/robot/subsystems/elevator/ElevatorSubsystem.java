package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public class ElevatorSubsystem {
    private ElevatorIO io;
    // put the little venderdeps akit file??? in the vednore deps
    private ElevatorIOInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
    // make the subsbsystem out of the talonfx class
    @AutoLog
    public void periodic() {
        io.updateInputs(inputs);
    }
}
