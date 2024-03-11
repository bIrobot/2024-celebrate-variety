// package frc.robot.autonomous.tasks;

// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.RobotContainer;

// public class DriveBackwardTask extends Task {
//     private final RobotContainer m_robotContainer;
//     private Timer m_timer = new Timer();

//     public DriveBackwardTask(RobotContainer robotContainer) {
//         m_robotContainer = robotContainer;
//     }

//     @Override
//     public void start() {
//         m_timer.start();
//     }

//     @Override
//     public void update() {
//         m_robotContainer.robotDrive.driveBackward();
//     }

//     @Override
//     public void done() {
//         m_timer.stop();
//         m_timer.reset();
//     }

//     @Override
//     public boolean isFinished() {
//         return m_timer.hasElapsed(2.25);
//     }
    
// }
