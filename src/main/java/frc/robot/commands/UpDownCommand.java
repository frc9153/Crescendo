// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.UpAndDownForever;

public class UpDownCommand extends Command {
  private final UpAndDownForever m_upDown;
  private final Constants.UpDownForever.Setpoint m_setpoint;

  public UpDownCommand(UpAndDownForever upDown, Constants.UpDownForever.Setpoint setpoint) {
    m_upDown = upDown;
    m_setpoint = setpoint;

    addRequirements(m_upDown);
  }

  @Override
  public void initialize() {
    m_upDown.gotoSetpoint(m_setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_upDown.isAtSetpoint();
  }
}
