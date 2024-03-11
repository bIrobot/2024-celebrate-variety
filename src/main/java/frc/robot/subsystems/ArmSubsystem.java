package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_armMotor;
    private final RelativeEncoder m_armEncoder;
    private final SparkPIDController m_armPIDController;
    private ArmState m_currentArmState = ArmState.STOP;

    private enum ArmState {
        LOWER,
        RAISE,
        STOP
    };

    public ArmSubsystem(int canID) {
        m_armMotor = new CANSparkMax(canID, MotorType.kBrushless);

        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setP(ArmConstants.kClimberP);
        m_armPIDController.setI(ArmConstants.kClimberI);
        m_armPIDController.setD(ArmConstants.kClimberD);
        m_armPIDController.setOutputRange(ArmConstants.kClimberMinOutput, ArmConstants.kClimberMaxOutput);

        m_armEncoder = m_armMotor.getEncoder();
        m_armEncoder.setPositionConversionFactor(ArmConstants.kClimberGearRatio);
        m_armEncoder.setVelocityConversionFactor(ArmConstants.kClimberGearRatio);

        m_armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void lowerArm() {
        m_currentArmState = ArmState.LOWER;
    }

    public void raiseArm() {
        m_currentArmState = ArmState.RAISE;
    }

    public void stopArm() {
        m_currentArmState = ArmState.STOP;
    }

    @Override
    public void periodic() {
        switch (m_currentArmState){
            case LOWER:
                m_armPIDController.setReference(ArmConstants.kClimberClimbSpeed, CANSparkMax.ControlType.kVelocity);
                break;
            case RAISE:
                m_armPIDController.setReference(ArmConstants.kClimberReleaseSpeed, CANSparkMax.ControlType.kVelocity);
                break;
            case STOP:
                m_armPIDController.setReference(0.0, CANSparkMax.ControlType.kVelocity);
                break;
        }
    }
}
