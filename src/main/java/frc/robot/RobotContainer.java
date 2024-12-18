package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IngestSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.events.EventTrigger;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer {
    public final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
    public final DriveSubsystem robotDrive = new DriveSubsystem();
    public final ArmSubsystem leftArm = new ArmSubsystem(13);
    public final ArmSubsystem rightArm = new ArmSubsystem(14);
    public final IngestSubsystem ingestModule = new IngestSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(ingestModule);

    private final Field2d field; 

    private final SendableChooser<Command> autoChooser;

    public Command getlowerarmCommand() {
       return new FunctionalCommand (() -> {},  // onInit
                                     () -> ingestModule.startIngesting(),  // onExecute
                                     (interrupted) -> {},  // onEnd
                                     () -> { return ingestModule.getIngestHasNote(); },  // isFinished
                                     ingestModule);                                  
    }

    public Command getallstopCommand() {
       return new FunctionalCommand (() -> { robotDrive.drive(0, 0, 0, false, false); },  // onInit
                                     () -> {},  // onExecute
                                     (interrupted) -> {},  // onEnd
                                     () -> { return true; },  // isFinished
                                     robotDrive);                                  
    }    

    public Command getshootCommand() {
       return new FunctionalCommand (() -> shooterSubsystem.startShooting(),  // onInit
                                     () -> {},  // onExecute
                                     (interrupted) -> {},  // onEnd
                                     () -> false,  // isFinished
                                     shooterSubsystem).
                                     withTimeout(5).
                                     andThen(Commands.runOnce(() -> shooterSubsystem.stopShooting())); 
    }

    public RobotContainer() {
        configureSwerveDrive();
        CameraServer.startAutomaticCapture();
        NamedCommands.registerCommand("lowerarm", getlowerarmCommand());
        NamedCommands.registerCommand(("shoot"), getshootCommand());
        NamedCommands.registerCommand(("allstop"), getallstopCommand());

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            //System.out.println("CURRENT = " + pose.getX() + " " + pose.getY() + " " + pose.getRotation());
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            //System.out.println("TARGET = " + pose.getX() + " " + pose.getY() + " " + pose.getRotation());
        });
       
        field = new Field2d(); 
        SmartDashboard.putData("Field2", field);
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path2").setPoses(poses);
        });

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }  

    // Calls the pathplanner file
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void teleopRunning() {
        shouldLeftArmChangeState();
        shouldRightArmChangeState();
        shouldStartIngesting();
        shouldStartShooting();
        shouldStartIngestPulse();
        shouldSetPivotAmp();

        if (driverController.getBackButtonPressed()) {
            //System.out.println("RESET POSE");
            robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }

        if (driverController.getStartButtonPressed()) {
            Pose2d pose = robotDrive.getPose();
            //System.out.println("POSE = " + pose.getX() + " " + pose.getY() + " " + pose.getRotation());
            ChassisSpeeds speeds = robotDrive.getSpeeds();
            //System.out.println("SPEEDS = " + speeds.vxMetersPerSecond + " " +
            //                                 speeds.vyMetersPerSecond + " " +
            //                                 speeds.omegaRadiansPerSecond);
            //System.out.println("GYRO = " + robotDrive.getHeading());
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
