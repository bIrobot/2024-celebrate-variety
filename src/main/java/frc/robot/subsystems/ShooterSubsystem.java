package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax mLeftShooterMotor;
    private CANSparkMax mRightShooterMotor;

    private SparkPIDController mLeftShooterPID;
    private SparkPIDController mRightShooterPID;

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(2000);
    private final IngestSubsystem m_ingestModule;
    private ShootingState m_currentShootingState = ShootingState.STOP;
    private Timer m_timer = new Timer();

    private enum ShootingState {
        AMP_SHOOT,
        AUTO_STOP,
        START,
        STOP
    };

    public ShooterSubsystem(IngestSubsystem ingestModule) {
        m_ingestModule = ingestModule;
        mLeftShooterMotor = new CANSparkMax(ShooterConstants.kShooterLeftMotorId, MotorType.kBrushless);
        mRightShooterMotor = new CANSparkMax(ShooterConstants.kShooterRightMotorId, MotorType.kBrushless);
        mLeftShooterMotor.restoreFactoryDefaults();
        mRightShooterMotor.restoreFactoryDefaults();
    
        mLeftShooterPID = mLeftShooterMotor.getPIDController();
        mLeftShooterPID.setP(ShooterConstants.kShooterP);
        mLeftShooterPID.setI(ShooterConstants.kShooterI);
        mLeftShooterPID.setD(ShooterConstants.kShooterD);
        mLeftShooterPID.setFF(ShooterConstants.kShooterFF);
        mLeftShooterPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
    
        mRightShooterPID = mRightShooterMotor.getPIDController();
        mRightShooterPID.setP(ShooterConstants.kShooterP);
        mRightShooterPID.setI(ShooterConstants.kShooterI);
        mRightShooterPID.setD(ShooterConstants.kShooterD);
        mRightShooterPID.setFF(ShooterConstants.kShooterFF);
        mRightShooterPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
    
        mLeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
        mLeftShooterMotor.setInverted(true);
        mRightShooterMotor.setInverted(false);
    }

    public void startShooting() {
        m_currentShootingState = ShootingState.START;
    }

    public void stopShooting() {
        m_currentShootingState = ShootingState.STOP;
    }

    public void startAmpShooting() {
        m_currentShootingState = ShootingState.AMP_SHOOT;
    }

    public void autoStopShooting() {
        m_currentShootingState = ShootingState.AUTO_STOP;
    }

    @Override
    public void periodic() {
        switch (m_currentShootingState){
            case START:
                double speed = MathUtil.clamp(6000, -6000, 10000);
                setShooterSpeed(speed);

                m_timer.start();

                if (m_timer.hasElapsed(3.5)) {
                    m_ingestModule.reverseIngest();
                }
                break;
            case AMP_SHOOT:
                double ampShootingSpeed = MathUtil.clamp(1000, -6000, 10000);
                setShooterSpeed(ampShootingSpeed);

                m_timer.start();

                if (m_timer.hasElapsed(2)) {
                    m_ingestModule.reverseIngest();
                }
                break;
            case AUTO_STOP:
                m_timer.stop();
                m_timer.reset();
                setShooterSpeed(0.0);
                break;
            case STOP:
                m_timer.stop();
                m_timer.reset();
                m_ingestModule.stopIngesting();
                setShooterSpeed(0.0);
                break;
        }
    }

    public void setShooterSpeed(double desiredSpeed) {
        double limitedSpeed = mSpeedLimiter.calculate(desiredSpeed);
        mLeftShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
        mRightShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
    }
}