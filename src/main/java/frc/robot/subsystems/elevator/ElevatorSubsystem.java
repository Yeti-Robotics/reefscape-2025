package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.RIO_BUS;
import static frc.robot.subsystems.elevator.ElevatorConfigs.talonFXConfigs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX primaryElevatorMotor;
    private final TalonFX secondaryElevatorMotor;
    private final DigitalInput magSwitch;
    public final MotionMagicVoltage magicRequest;

    public ElevatorSubsystem() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigs.primaryElevatorMotorID, RIO_BUS);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigs.secondaryElevatorMotorID, RIO_BUS);

        primaryElevatorMotor.getConfigurator().apply(talonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfigs.primaryElevatorMotorID, false));

        magSwitch = new DigitalInput(ElevatorConfigs.magSwitchID);
        magicRequest = new MotionMagicVoltage(0);
    }

    public void setPosition(ElevatorPosition position) {
        primaryElevatorMotor.setControl(magicRequest.withPosition(position.getHeight()));
    }

    public Command tuning(double output) {
        return startEnd(
                () -> primaryElevatorMotor.setControl(new TorqueCurrentFOC(output)), this::stop);
    }

    public void stop() {
        primaryElevatorMotor.stopMotor();
    }

    public boolean getMagSwitch() {
        return magSwitch.get();
    }

    @Override
    public void periodic() {
        // Update logic if needed
    }
}
