// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.modes.AutoMode;
import frc.robot.autonomous.tasks.Task;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Task m_currentTask;
  private AutoMode m_autoRunner;

  private boolean m_tele;
  private long m_nanoTime;
  private int m_prints = 0;
  private double m_initX;
  private double m_initY;
  private double m_initRot;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_autoRunner = new AutoMode(m_robotContainer);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (m_tele && m_prints < 10 && System.nanoTime() - m_nanoTime > 1000000000L) {
      Pose2d pose = m_robotContainer.robotDrive.getPose();
      System.out.println("GETPOSE2 " + (pose.getX()-m_initX) + " " + (pose.getY()-m_initY) + " " + (pose.getRotation().getDegrees()-m_initRot));
      m_nanoTime = System.nanoTime();
      m_prints++;
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      System.out.println("WE HAVE A Command!!!");
      m_autonomousCommand.schedule();
    }
      // m_autoRunner.autoInit();
      // m_currentTask = m_autoRunner.getNextTask();

      // // Start the first task
      // if (m_currentTask != null) {
      //     m_currentTask.start();
      // }
  }

  @Override
  public void autonomousPeriodic() {
      // if (m_currentTask == null) {
      //   return;
      // }

      // m_currentTask.update();

      // if (!m_currentTask.isFinished()) {
      //   return;
      // }

      // // If the current task is finished, get the next task
      // m_currentTask.done();
      // m_currentTask = m_autoRunner.getNextTask();

      // // Start the next task
      // if (m_currentTask != null) {
      //   m_currentTask.start();
      // }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_nanoTime = System.nanoTime();
    m_tele = true;

    Pose2d pose = m_robotContainer.robotDrive.getPose();
    m_initX = pose.getX();
    m_initY = pose.getY();
    m_initRot = pose.getRotation().getDegrees();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopRunning();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
