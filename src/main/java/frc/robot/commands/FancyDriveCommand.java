// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.utils.Vector2;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FancyDriveCommand extends Command {
  static final double EPSILON = 0.01;
  // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DriveSwerve m_drive;
  private Vector2 m_targetOffset;

  public FancyDriveCommand(DriveSwerve driveSwerve, Vector2 targetOffset) {
    m_drive = driveSwerve;
    m_targetOffset = targetOffset;

    addRequirements(driveSwerve);
  }

  private Vector2 currentOffset() {
    // Because the rio and navex are not aligned with wheels, I think this should be (-Y, X)
    return Vector2.Zero();
    // return new Vector2(gyro.getDisplacementX(), gyro.getDisplacementY());
  }

  @Override
  public void initialize() {
    // gyro.resetDisplacement();
  }

  @Override public void execute() {
    // Double-checked, and all this math should be correct - Cedric
    Vector2 weAreHere = currentOffset();
    SmartDashboard.putString("we're here", weAreHere.toString());
    Vector2 targetDirection = weAreHere.minus(m_targetOffset).normalized().multBy(-1.0);
    SmartDashboard.putString("lets go", targetDirection.toString());
    m_drive.drive(targetDirection.multBy(0.1), 0, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(Vector2.Zero(), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return currentOffset().distanceTo(m_targetOffset) < EPSILON || currentOffset().magnitude() > 1.2 * ;
    return currentOffset().magnitude() >= m_targetOffset.magnitude(); // This condition does not seem correct - Cedric
  }
}
