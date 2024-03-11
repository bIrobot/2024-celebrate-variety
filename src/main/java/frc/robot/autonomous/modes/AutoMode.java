package frc.robot.autonomous.modes;

import java.util.ArrayList;

import frc.robot.RobotContainer;
import frc.robot.autonomous.tasks.DoNothingTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.PivotToGroundTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.Task;
import frc.robot.autonomous.tasks.WaitTask;

public class AutoMode {
    private ArrayList<Task> m_tasks;
    private RobotContainer m_robotContainer;

    public AutoMode(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    public void autoInit() {
        m_tasks = new ArrayList<>();
        m_tasks.add(new ShooterTask(m_robotContainer));
        m_tasks.add(new WaitTask(1));
        m_tasks.add(new PivotToGroundTask(m_robotContainer));
        m_tasks.add(new WaitTask(10));
        m_tasks.add(new DriveForwardTask(m_robotContainer, 1, 0.4));
        m_tasks.add(new WaitTask(0.1));
        m_tasks.add(new DriveForwardTask(m_robotContainer, 1, -0.4));
        m_tasks.add(new ShooterTask(m_robotContainer));
        m_tasks.add(new DoNothingTask(m_robotContainer));
    }

    public Task getNextTask() {
        // Pop the first task off the list and return it
        try {
            return m_tasks.remove(0);
        } catch (IndexOutOfBoundsException ex) {
            return null;
        }
    }
}
