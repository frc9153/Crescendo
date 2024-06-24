// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Esophagus;

public class KidSafeWindThenScore extends Command {
  Archerfish m_archerfish;
  Esophagus m_esophagus;

  public KidSafeWindThenScore(Archerfish archerfish, Esophagus esophagus) {
    m_archerfish = archerfish;
    m_esophagus = esophagus;
    
    addRequirements(archerfish, esophagus);
  }

  @Override
  public void initialize() {
    m_archerfish.childSpin();
  }

  @Override
  public void execute() {
    if (m_archerfish.isAtSpeed()) {
        m_esophagus.startFeeding();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_esophagus.stopFeeding();
    if (m_archerfish.isAtSpeed()) {
        m_archerfish.stopSpin();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
