// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Esophagus;
import frc.robot.Constants;

public class AmpThenScore extends Command {
  UpAndDownForever m_upDown;
  Archerfish m_archerfish;
  Esophagus m_esophagus;
  private long start_time;

  public AmpThenScore(UpAndDownForever upDown, Archerfish archerfish, Esophagus esophagus) {
    m_upDown = upDown;
    m_archerfish = archerfish;
    m_esophagus = esophagus;
    
    addRequirements(upDown, archerfish, esophagus);
  }

  @Override
  public void initialize() {
    m_upDown.gotoSetpoint(Constants.UpDownForever.Setpoint.AMP);
    start_time = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (m_upDown.isAtSetpoint()) {
        if (start_time != -1) {
            System.out.print("Arm travel time: ");
            System.out.println(System.currentTimeMillis()-start_time);
            start_time = -1;
        }
        m_esophagus.startFeeding();
        m_archerfish.slowSpin();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_esophagus.stopFeeding();
    m_archerfish.stopSpin();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
