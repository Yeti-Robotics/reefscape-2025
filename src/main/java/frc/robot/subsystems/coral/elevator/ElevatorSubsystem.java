package frc.robot.subsystems.coral.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.state.StatefulSetpointSubsystem;

import static frc.robot.constants.Constants.RIO_BUS;
import static frc.robot.subsystems.coral.elevator.ElevatorConfig.primaryTalonFXConfigs;
import static frc.robot.subsystems.coral.elevator.ElevatorConfig.secondaryTalonFXConfigs;

public class ElevatorSubsystem extends StatefulSetpointSubsystem<ElevatorPosition, AngleUnit, Angle, MutAngle> {
    private final TalonFX primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID, RIO_BUS);
    private final TalonFX secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID, RIO_BUS);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);

    private final MotionMagicVoltage motionVoltageRequest = new MotionMagicVoltage(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();

    public ElevatorSubsystem() {
        super(ElevatorPosition.HOLD, new MutAngle(0, 0, Units.Rotations), Units.Rotations.of(ElevatorConfig.HEIGHT_TOLERANCE));
        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, true));

        new Trigger(this::getMagSwitch).onTrue(zeroPosition().andThen(transitionTo(ElevatorPosition.BOTTOM)));
    }

    private Command zeroPosition() {
        return runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean getMagSwitch() {
        return magSwitch.get();
    }

    @Override
    public StatusSignal<Angle> currentStateSignal() {
        return elevatorPosition;
    }

    @Override
    public Angle determineSetpoint(ElevatorPosition targetState) {
        return targetState == ElevatorPosition.HOLD ? elevatorPosition.getValue() : targetState.getHeight();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return primaryElevatorMotor.setControl(motionVoltageRequest.withPosition(setpoint));
    }
}
