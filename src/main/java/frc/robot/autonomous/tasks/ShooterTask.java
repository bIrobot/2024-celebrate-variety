package frc.robot.autonomous.tasks;

import frc.robot.RobotContainer;

public class ShooterTask extends Task {
    private final RobotContainer m_robotContainer;

    public ShooterTask(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    @Override
    public void start() {
        m_robotContainer.shooterSubsystem.startShooting();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        m_robotContainer.shooterSubsystem.autoStopShooting();
    }

    @Override
    public boolean isFinished() {
        return !m_robotContainer.ingestModule.getIngestHasNote();
    }
}
