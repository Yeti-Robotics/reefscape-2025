package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.util.akit.device.can.CANUtil;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;
import frc.robot.util.akit.device.digital.DigitalInputDevice;

import java.util.function.Supplier;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFXDevice primaryElevatorMotor = TalonFXDevice.configure(
                    ElevatorConfig.primaryElevatorMotorID, Constants.CANIVORE_BUS)
            .log("Elevator/PrimaryMotor")
            .withConfig(ElevatorConfig.primaryTalonFXConfigs)
            .syncConfigs();

    private final Supplier<Angle> elevatorPosition = primaryElevatorMotor.positionSupplier(CANUtil.TALON_MAX_UPDATE_HZ);
    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);

    private final Trigger zeroSwitch = DigitalInputDevice.configure(ElevatorConfig.magSwitchID)
            .log("Elevator/MagSwitch")
            .toTrigger();

    public ElevatorIOTalonFX() {
        TalonFXDevice.configure(
                        ElevatorConfig.secondaryElevatorMotorID, Constants.CANIVORE_BUS)
                .log("Elevator/SecondaryMotor")
                .withConfig(ElevatorConfig.primaryTalonFXConfigs)
                .modifyConfig(config -> config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive)
                .syncConfigs()
                .oppose(primaryElevatorMotor.getDevice());
    }

    @Override
    public Angle getState() {
        return elevatorPosition.get();
    }

    @Override
    public void setState(Angle value) {
        primaryElevatorMotor.getDevice().setControl(motionMagicReq.withPosition(value));
    }

    @Override
    public Trigger zeroTrigger() {
        return zeroSwitch;
    }
}
