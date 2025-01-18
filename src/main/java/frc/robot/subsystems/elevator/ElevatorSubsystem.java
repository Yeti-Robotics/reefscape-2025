package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    private final TalonFX primaryElevatorMotor;
    //private final TalonFX secondaryElevatorMotor;

    // Sensors
    private final Encoder encoder;
    private final DigitalInput magSwitch;

    public final MotionMagicVoltage magicRequest;

    // Constants
    private static final int TICKS_PER_INCH = 1000;
    private static final int STAGE_RATIO = 2;


    public ElevatorSubsystem() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigs.primaryElevatorMotorPort, "CANivore");

        // secondaryElevatorMotor = new TalonFX(ElevatorConfigs.secondaryElevatorMotorPort);
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Slot0.kP = ElevatorConfigs.Elevator_kP;
        talonFXConfigs.Slot0.kI = ElevatorConfigs.Elevator_kI;
        talonFXConfigs.Slot0.kD = ElevatorConfigs.Elevator_kD;

        primaryElevatorMotor.getConfigurator().apply(talonFXConfigs);

        // Secondary motor follows the primary motor
        //secondaryElevatorMotor.setControl(new Follower(ElevatorConfigs.primaryElevatorMotorPort, true));

        // Initialize encoder
        encoder = new Encoder(ElevatorConfigs.encoderChannelA, ElevatorConfigs.encoderChannelB);

        // Initialize magnetic switch
        magSwitch = new DigitalInput(ElevatorConfigs.magSwitchPort);

        // Reset encoder on initialization
        encoder.reset();

        // Initialize Motion Magic request
        magicRequest = new MotionMagicVoltage(0);

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1;
        motionMagicConfigs.MotionMagicAcceleration = 1;
        motionMagicConfigs.MotionMagicJerk = 1;
    }

    // Sets elevator position to a specific height
    public void setPosition(ElevatorConfigs.ElevatorPosition position) {
        double targetTicks = position.getHeight() * TICKS_PER_INCH / STAGE_RATIO;
        primaryElevatorMotor.setControl(magicRequest.withPosition(targetTicks));
    }

    // Stops the elevator
    public void stop() {
        primaryElevatorMotor.stopMotor();
    }

    // Checks magnetic switch status
    public boolean getMagSwitch() {
        return magSwitch.get();
    }

    @Override
    public void periodic() {
        // Update logic if needed
    }
}
