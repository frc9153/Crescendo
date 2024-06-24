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
  private final BooleanSupplier m_canRun;

  public ManualUpDownCommand(UpAndDownForever upDown, DoubleSupplier speed, BooleanSupplier canRun) {
    m_upDown = upDown;
    m_speed = speed;
    m_canRun = canRun;

    addRequirements(m_upDown);
  }

  @Override
  public void initialize() {
    if (m_canRun) {
        m_upDown.setSpeed(m_speed.getAsDouble());
    }else {
        // This constant was not pushed on the computer I built this on.
        // Implement this later
        system.out.println("Implement me! (BooleanTriggeredManualUpDownCommand, stopfromFallingSpeed)");
        //m_upDown.setSpeed(Constants.UpAndDownForever.stopFromFalligSpeed);
    }
  }

  @Override
  public void execute() {
    if (m_canRun) {
        m_upDown.setSpeed(m_speed.getAsDouble());
    }else {
        // This constant was not pushed on the computer I built this on.
        // Implement this later
        system.out.println("Implement me! (BooleanTriggeredManualUpDownCommand, stopfromFallingSpeed)");
        //m_upDown.setSpeed(Constants.UpAndDownForever.stopFromFalligSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_upDown.killItNow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_upDown.isInBounds();

    // If we are using this, clearly setpoints are not working, therefore,
    // we cannot trust the absolute encoder at all. If we limit it, it could
    // be one of those limits which malfunctions.
    // THIS IS VERY DANGEROUS!!!!!
    return false;
  }
}
