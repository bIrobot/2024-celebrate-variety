package frc.robot.autonomous.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;

public class DriveForwardTask extends Task {
    private final RobotContainer m_robotContainer;

    private double m_targetDistance;
    private double m_speed;
    private Pose2d m_startPose;

    public DriveForwardTask(RobotContainer robotContainer, double distance, double speed) {
        m_robotContainer = robotContainer;
        m_targetDistance = distance;
        m_speed = speed;
    }

    // @Override
    public void start() {
        m_startPose = m_robotContainer.robotDrive.getPose();
    }

    @Override
    public void update() {
        m_robotContainer.robotDrive.drive(m_speed, 0, 0, true, true);
    }

    @Override
    public void done() {
        m_robotContainer.robotDrive.doNothing();
    }

    @Override
    public boolean isFinished() {
        Pose2d relativePose = m_startPose.relativeTo(m_robotContainer.robotDrive.getPose());
        return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
    }
    
}
