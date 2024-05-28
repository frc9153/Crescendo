// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.UpAndDownForever;

public class ManualUpDownCommand extends Command {
  private final UpAndDownForever m_upDown;
  private final DoubleSupplier m_speed;

  public ManualUpDownCommand(UpAndDownForever upDown, DoubleSupplier speed) {
    m_upDown = upDown;
    m_speed = speed;

    addRequirements(m_upDown);
  }

  @Override
  public void initialize() {
    m_upDown.setSpeed(m_speed.getAsDouble());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_upDown.killItNow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_upDown.isInBounds();
  }
}
