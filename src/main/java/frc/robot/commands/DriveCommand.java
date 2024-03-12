// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.utils.Vector2;

public class DriveCommand extends Command {
  DriveSwerve m_drive;
  Vector2 m_speed;
  double m_rotation;
  boolean m_isFieldRelative;

  public DriveCommand(DriveSwerve drive, Vector2 speed, double rotation, boolean isFieldRelative) {
    m_drive = drive;
    m_speed = speed;
    m_rotation = rotation;
    m_isFieldRelative = isFieldRelative;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.drive(m_speed, m_rotation, m_isFieldRelative);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(Vector2.Zero(), 0.0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
