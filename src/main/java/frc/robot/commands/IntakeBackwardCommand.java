package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Esophagus;

public class IntakeBackwardCommand extends Command {
  Esophagus m_esophagus;

  public IntakeBackwardCommand(Esophagus esophagus) {
    m_esophagus = esophagus;
    
    addRequirements(m_esophagus);
  }

  @Override
  public void initialize() {
    m_esophagus.startReverse();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_esophagus.stopFeeding();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Esophagus;

public class IntakeBackwardCommand extends Command {
  Esophagus m_esophagus;

  public IntakeBackwardCommand(Esophagus esophagus) {
    m_esophagus = esophagus;
    
    addRequirements(m_esophagus);
  }

  @Override
  public void initialize() {
    m_esophagus.startReverse();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_esophagus.stopFeeding();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
