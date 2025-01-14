package frc.robot;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
    // Motors
    private final TalonFX primaryElevatorMotor;
    private final TalonFX secondaryElevatorMotor;

    // Sensors
    private final Encoder encoder;
    private final DigitalInput magSwitch;

    // Constants
    private static final int TICKS_PER_INCH = 1000;
    private static final int STAGE_RATIO = 2;

    public enum ElevatorPosition {
        BOTTOM(0.0),
        MIDDLE(24.0),
        TOP(48.0);

        private final double height;

        ElevatorPosition(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    public ElevatorSubsystem(int primaryElevatorMotorPort, int secondaryElevatorMotorPort, int encoderChannelA, int encoderChannelB, int magSwitchPort) {
        primaryElevatorMotor = new TalonFX(primaryElevatorMotorPort);
        secondaryElevatorMotor = new TalonFX(secondaryElevatorMotorPort);

        // Follows primary motor
        secondaryElevatorMotor.setControl(new Follower(primaryElevatorMotorPort, true));

        // Initialize encoder
        encoder = new Encoder(encoderChannelA, encoderChannelB);

        // Initialize magnetic switch
        magSwitch = new DigitalInput(magSwitchPort);

        // Reset encoder on initialization
        encoder.reset();

    }


    @Override
    public void periodic() {
        // Periodic updates will be added soon
    }
}


