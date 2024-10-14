package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IngestConstants;
import frc.utils.SwerveUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IngestSubsystem extends SubsystemBase{
    private final CANSparkMax m_ingestMotor;
    private CANSparkMax mPivotMotor;
    private RelativeEncoder m_relativePivotEncoder;
    private final SparkPIDController m_pivotPID;
    private IngestState m_currentIngestState = IngestState.STOP;
    private PivotState m_currentPivotState = PivotState.STOW;
    private final DigitalInput m_ingestLimitSwitch = new DigitalInput(2);
    private static final double k_pivotMotorP = 0.12;
    private static final double k_pivotMotorI = 0.0;
    private static final double k_pivotMotorD = 0.001;
    private static double m_pulseSpeed = 0.25;

    private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(0);

    private enum IngestState {
        FORWARD,
        REVERSE,
        STOP,
        PULSE
    };

    private enum PivotState {
        GROUND,
        STOW
    };
  
    public IngestSubsystem() {
        m_ingestMotor = new CANSparkMax(9, MotorType.kBrushless);
        m_ingestMotor.restoreFactoryDefaults();
        m_ingestMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        mPivotMotor = new CANSparkMax(10, MotorType.kBrushless);

        m_pivotPID = mPivotMotor.getPIDController();
        m_pivotPID.setP(k_pivotMotorP);
        m_pivotPID.setI(k_pivotMotorI);
        m_pivotPID.setD(k_pivotMotorD);
        m_pivotPID.setOutputRange(-1.0, 1.0);

        m_relativePivotEncoder = mPivotMotor.getEncoder();
        m_relativePivotEncoder.setPositionConversionFactor(ArmConstants.kClimberGearRatio);
        m_relativePivotEncoder.setVelocityConversionFactor(ArmConstants.kClimberGearRatio);

        mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public boolean getIngestHasNote() {
        // NOTE: this is intentionally inverted, because the limit switch is normally
        // closed
        return !m_ingestLimitSwitch.get();
    }

    public void startIngesting() {
        m_currentIngestState = IngestState.FORWARD;
        m_currentPivotState = PivotState.GROUND;
    }

    public void reverseIngest() {
        m_currentIngestState = IngestState.REVERSE;
        m_currentPivotState = PivotState.STOW;
    }

    public void stopIngesting() {
        m_currentIngestState = IngestState.STOP;
        m_currentPivotState = PivotState.STOW;
    }

    public void pulseIngest() {
        m_currentIngestState = IngestState.PULSE;
    }

    public boolean isPivotAtTarget() {
        int deadzoneBuffer = 40;
        return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle()) < deadzoneBuffer;
    }

    @Override
    public void periodic() {
        // UNCOMMENT LINE 97 TO GET DEGREES FOR PIVOT STATES
        // System.out.println(getPivotAngleDegrees());

        // COMMENT OUT LINES 100-106 if you uncomment line 97.
        if (getIngestHasNote() && m_currentPivotState == PivotState.GROUND && m_currentIngestState == IngestState.FORWARD) {
            m_currentIngestState = IngestState.STOP;
            m_currentPivotState = PivotState.STOW;
        }

        setIngestMotor();
        setPivotAngle();
    }

    private void setIngestMotor() {
        switch (m_currentIngestState){
            case FORWARD:
                m_ingestMotor.set(0.25);
                break;
            case REVERSE:
                m_ingestMotor.set(-0.5);
                break;
            case STOP:
                m_ingestMotor.set(0.0);
                break;
            case PULSE:

                // Use the timer to pulse the intake on for a 1/16 second,
                // then off for a 15/16 second
                if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
                    m_ingestMotor.set(m_pulseSpeed);
                    return;
                }

                m_ingestMotor.set(0.0);
            default:
                m_ingestMotor.set(0.0);
                break;
        }
    }

    private void setPivotAngle() {
        if (isPivotAtTarget()) {
            mPivotMotor.set(0.0);
            return;
        }

        switch (m_currentPivotState) {
          case GROUND:
            mPivotMotor.set(-.6);
            break;
          case STOW:
            mPivotMotor.set(.6);
            break;
          default:
            mPivotMotor.set(0.0);
            break;
        }
    }

    private double pivotTargetToAngle() {
        switch (m_currentPivotState) {
          case GROUND:
            return IngestConstants.k_pivotAngleGround;
          case STOW:
            return IngestConstants.k_pivotAngleStow;
          default:
            return 180;
        }
    }

    public double getPivotAngleDegrees() {
        double value = m_pivotEncoder.getAbsolutePosition() - IngestConstants.k_pivotEncoderOffset + 0.5;
        return Units.rotationsToDegrees(SwerveUtils.ModRotations(value));
    }
}
