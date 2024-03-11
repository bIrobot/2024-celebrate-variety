package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IngestSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    public final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
    public final DriveSubsystem robotDrive = new DriveSubsystem();
    public final ArmSubsystem leftArm = new ArmSubsystem(13);
    public final ArmSubsystem rightArm = new ArmSubsystem(14);
    public final IngestSubsystem ingestModule = new IngestSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(ingestModule);

    public RobotContainer() {
        configureSwerveDrive();
        CameraServer.startAutomaticCapture();
    }  

    public void teleopRunning() {
        shouldLeftArmChangeState();
        shouldRightArmChangeState();
        shouldStartIngesting();
        shouldStartShooting();
        shouldStartIngestPulse();
        shouldSetPivotAmp();
    }
        
    private void configureSwerveDrive() {
            // Configure default commands
        robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    true),
                robotDrive));
    }

    private void shouldLeftArmChangeState() {
        if (driverController.getLeftBumper()) {
            leftArm.raiseArm();
        }
        else if (driverController.getLeftTriggerAxis() > .1) {
            leftArm.lowerArm();
        }
        else if (!driverController.getLeftBumper() || driverController.getLeftTriggerAxis() <= .1) {
            leftArm.stopArm();
        }
    }

    private void shouldRightArmChangeState() {
        if (driverController.getRightBumper()) {
            rightArm.raiseArm();
        }
        else if (driverController.getRightTriggerAxis() > .1) {
            rightArm.lowerArm();
        }
        else if (!driverController.getRightBumper() || driverController.getRightTriggerAxis() <= .1) {
            rightArm.stopArm();
        }
    }

    private void shouldStartIngesting() {
        if (driverController.getXButton()) {
        ingestModule.startIngesting();
        }
        else if (driverController.getXButtonReleased()){
        ingestModule.stopIngesting();
        }
    }

    private void shouldSetPivotAmp() {
        if (driverController.getBButton()) {
        shooterSubsystem.startAmpShooting();
        }
        else if (driverController.getBButtonReleased()){
        shooterSubsystem.stopShooting();
        }
    }

    private void shouldStartIngestPulse() {
        if (driverController.getYButton()) {
        ingestModule.pulseIngest();
        }
        else if (driverController.getYButtonReleased()){
        ingestModule.stopIngesting();
        }
    }

    private void shouldStartShooting() {
        if (driverController.getAButtonPressed()){
        shooterSubsystem.startShooting();
        }
        else if (driverController.getAButtonReleased()){
        shooterSubsystem.stopShooting();
        }
    }
}
