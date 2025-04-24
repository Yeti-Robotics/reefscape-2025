package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.coral.elevator.ElevatorPosition;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;
import frc.robot.util.akit.device.digital.DigitalInputDevice;

public class ElevatorIOTalonFX implements ElevatorIO {
    protected final TalonFX primaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.primaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/PrimaryMotor")
                    .withConfig(ElevatorConfig.primaryTalonFXConfigs)
                    .withStatusSignalFrequency(
                            HardwareConstants.SETPOINT_UPDATE_FREQUENCY, TalonFX::getPosition)
                    .syncConfigs()
                    .getDevice();

    protected final TalonFX secondaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.secondaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/SecondaryMotor")
                    .withConfig(ElevatorConfig.secondaryTalonFXConfigs)
                    .syncConfigs()
                    .oppose(primaryElevatorMotor)
                    .getDevice();

    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();

    private final DigitalInput magSwitch;

    public ElevatorIOTalonFX() {
        DigitalInputDevice digitalSwitch = DigitalInputDevice.configure(ElevatorConfig.magSwitchID);

        if (Robot.isSimulation()) {
            SimDevice deviceSim = SimDevice.create("Elevator Switch", ElevatorConfig.magSwitchID);

            if (deviceSim != null) {
                digitalSwitch.usingSimDevice(deviceSim);
            }
        }

        magSwitch = digitalSwitch.getDevice();

        digitalSwitch
                .toTrigger()
                .debounce(2)
                .onTrue(
                        Commands.runOnce(() -> primaryElevatorMotor.setPosition(0))
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
