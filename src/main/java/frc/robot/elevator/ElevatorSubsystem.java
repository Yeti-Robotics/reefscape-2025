package frc.robot.elevator;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.Constants.RIO_BUS;

public class ElevatorSubsystem extends SubsystemBase {
    final TalonFX elevatorMotor;
    public final MotionMagicVoltage magicRequest;


    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConfigs.ELEVATOR_ID, RIO_BUS);
        var elevatorConfigurator = elevatorMotor.getConfigurator();
        var talonFXConfiguration = new TalonFXConfiguration();

        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = ElevatorConfigs.ELEVATOR_ID;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfiguration.FutureProofConfigs = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = 0;
        talonFXConfiguration.Feedback.RotorToSensorRatio = 0;
        talonFXConfiguration.Slot0 = ElevatorConfigs.SLOT_0_CONFIGS;

        magicRequest = new MotionMagicVoltage(0);

    }

    public void setPosition(double position){
        elevatorMotor.setControl(magicRequest.withPosition(position));

    }

    public Command setPositionTo(double position){
        return runOnce(() -> setPosition(position));
    }
}

