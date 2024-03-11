package frc.robot.autonomous.tasks;

public abstract class Task {
  public abstract void start();

  public abstract void done();

  public abstract void update();

  public abstract boolean isFinished();
}