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

    boolean seeking = false;
    double lastTX;

    public RobotContainer() {
        configureSwerveDrive();
        CameraServer.startAutomaticCapture();
    }  

    public void teleopRunning() {

        if (driverController.getStartButtonPressed()){
            seeking = ! seeking;
        }
        if (! seeking) {
            shouldLeftArmChangeState();
            shouldRightArmChangeState();
            shouldStartIngesting();
            shouldStartShooting();
            shouldStartIngestPulse();
            shouldSetPivotAmp();
        } else {
            System.out.println(LimelightHelpers.getTA("limelight"));
            double TA, TX;
            TA=LimelightHelpers.getTA("limelight");
            TX=LimelightHelpers.getTX("limelight");
            
            if (TA != 0) { 
                if (TX != 0) {
                    lastTX=TX;
                }
                if (Math.abs(TX)<5) {
                    if (TA > 1) {
                        System.out.println("Stop " + TA);
                        robotDrive.drive(0, 0, 0, false, true);
                    } else {
                        System.out.println("Go forward " + TA);
                        robotDrive.drive(-0.1, 0, 0, false, true);
                    }
                } else if (TX<0) {
                    System.out.println("Rotate left slow"); 
                    robotDrive.drive(0, 0, 0.1, false, true);
                } else {
                    System.out.println("Right Slow");
                    robotDrive.drive(0, 0, -0.1, false, true);
                }
            } else {
                System.out.println("Dont see");
                if (lastTX<0) {
                    robotDrive.drive(0, 0, 0.2, false, true);
                } else {
                    robotDrive.drive(0, 0, -0.2, false, true);
                }
            }

        }
        
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
                    false,
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
