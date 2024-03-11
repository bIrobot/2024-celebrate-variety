package frc.robot.autonomous.tasks;

import frc.robot.RobotContainer;

public class PivotToGroundTask extends Task {
    private final RobotContainer m_robotContainer;

    public PivotToGroundTask(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    @Override
    public void start() {
        m_robotContainer.shooterSubsystem.autoStopShooting();
        m_robotContainer.ingestModule.startIngesting();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public boolean isFinished() {
        return m_robotContainer.ingestModule.isPivotAtTarget();
    }
    
}
