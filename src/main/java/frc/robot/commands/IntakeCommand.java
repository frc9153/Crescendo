package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Esophagus;

public class IntakeCommand extends Command {
  Esophagus m_esophagus;

  public IntakeCommand(Esophagus esophagus) {
    m_esophagus = esophagus;
    
    addRequirements(m_esophagus);
  }

  @Override
  public void initialize() {
    m_esophagus.startFeeding();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_esophagus.stopFeeding();
    if (!interrupted) {
        System.out.println("Sensor Tripped!");
    }
  }

  @Override
  public boolean isFinished() {
    return m_esophagus.SensorTriggered();
  }
}
