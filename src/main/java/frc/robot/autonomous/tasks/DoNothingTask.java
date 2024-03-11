package frc.robot.autonomous.tasks;

import frc.robot.RobotContainer;

public class DoNothingTask extends Task {
    private final RobotContainer m_robotContainer;

    public DoNothingTask(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    @Override
    public void start() {
        m_robotContainer.ingestModule.stopIngesting();
        m_robotContainer.robotDrive.doNothing();
        m_robotContainer.shooterSubsystem.stopShooting();
    }

    @Override
    public void done() {
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
