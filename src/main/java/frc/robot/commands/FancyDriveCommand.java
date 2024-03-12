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

public class FancyDriveCommand extends Command {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DriveSwerve m_drive;
  private double m_distance;
  private double m_target_angle;
  Vector2 m_speed;
  double m_rotation;
  boolean m_isFieldRelative;
  double m_rotation_sign;

  Vector2 speed;
  double rotation;

  public FancyDriveCommand(DriveSwerve driveSwerve, Vector2 speed, double rotation, boolean isFieldRelative, double distance, double angle) {
    m_drive = driveSwerve;
    m_speed = speed;
    m_rotation = rotation;
    m_isFieldRelative = isFieldRelative;
    m_distance = distance;
    m_target_angle = angle;

    addRequirements(driveSwerve);
  }

  @Override
  public void initialize() {
    speed = m_speed;
    rotation = m_rotation;

    System.out.print("Before: ");
    System.out.print(gyro.getDisplacementX());
    System.out.println(", "+gyro.getDisplacementY());
    gyro.resetDisplacement();
    System.out.print("After: ");
    System.out.print(gyro.getDisplacementX());
    System.out.println(", "+gyro.getDisplacementY());

    if (m_target_angle == 0.0) {
      m_rotation_sign = 0.0;
      m_rotation = 0.0;
    }else {
      m_target_angle += gyro.getAngle() + m_target_angle;

      if (m_target_angle-gyro.getAngle() > 0.0) {
        m_rotation_sign = 1.0;
      }else if (m_target_angle-gyro.getAngle() < 0.0) {
        m_rotation_sign = -1.0;
      }
    }

    if (m_distance == 0.0) {
      m_speed = Vector2.Zero();
    }

    m_drive.drive(m_speed, m_rotation, m_isFieldRelative);
  }

  @Override
  public void execute() {
    speed = m_speed;
    rotation = m_rotation;

    if ((m_target_angle-gyro.getAngle() >= 0.0 && m_rotation_sign < 0.0) || 
        (m_target_angle-gyro.getAngle() <= 0.0 && m_rotation_sign > 0.0) || 
        m_rotation_sign == 0.0) {
      System.out.println("Oriented!!!!!!!!!!!!!!!!!!");
      rotation = 0.0;
    }
    System.out.print(gyro.getDisplacementX());
    System.out.println(", "+gyro.getDisplacementY());
    System.out.println(Math.sqrt(Math.pow(gyro.getDisplacementX(), 2.0)+Math.pow(gyro.getDisplacementY(), 2.0)));
    if (Math.sqrt(Math.pow(gyro.getDisplacementX(), 2.0)+Math.pow(gyro.getDisplacementY(), 2.0)) > m_distance) {
      System.out.println("We're Far Enough!!!!!!!!!!!!!!!!!!!!");
      speed = Vector2.Zero();
    }

    m_drive.drive(speed, rotation, m_isFieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("We're Here!!!!!!!!!!!!!!!!!!!!");
    m_drive.drive(Vector2.Zero(), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (rotation == 0.0 && speed.magnitude() == 0);
  }
}
