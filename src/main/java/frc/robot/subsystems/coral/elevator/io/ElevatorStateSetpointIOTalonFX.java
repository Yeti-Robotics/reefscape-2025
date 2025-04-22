package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class ElevatorStateSetpointIOTalonFX implements ElevatorStateSetpointIO {
    protected final TalonFX primaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.primaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/PrimaryMotor")
                    .using(ElevatorConfig.primaryTalonFXConfigs)
                    .withStatusSignalFrequency(
                            HardwareConstants.SETPOINT_UPDATE_FREQUENCY, TalonFX::getPosition)
                    .syncConfigs()
                    .getDevice();

    protected final TalonFX secondaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.secondaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/SecondaryMotor")
                    .using(ElevatorConfig.secondaryTalonFXConfigs)
                    .syncConfigs()
                    .oppose(primaryElevatorMotor)
                    .getDevice();

    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();

    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);

    public ElevatorStateSetpointIOTalonFX() {
        new Trigger(this::bottomSwitchTriggered)
                .debounce(2)
                .onTrue(Commands.runOnce(() -> primaryElevatorMotor.setPosition(0))
                        .andThen(() -> toSetpoint(ElevatorPosition.BOTTOM))
                        .andThen(primaryElevatorMotor::stopMotor));
    }

    @Override
    public Angle getState() {
        return elevatorPosition.getValue();
    }

    @Override
    public void setState(Angle value) {
        primaryElevatorMotor.setControl(motionMagicReq.withPosition(value));
    }

    @Override
    public boolean bottomSwitchTriggered() {
        return magSwitch.get();
    }
}
