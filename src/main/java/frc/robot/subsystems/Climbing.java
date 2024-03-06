// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbing extends SubsystemBase {
  // just like that swedish game "Climbing"
  private CANSparkBase m_leftClimbMotor;
  private CANSparkBase m_rightClimbMotor;

  public Climbing() {
    // i swear on my left foot that there was a class to control a pair of motors in tandem but its escaping me
    m_leftClimbMotor = new CANSparkMax(Constants.Climbing.leftMotorId, MotorType.kBrushless);
    m_rightClimbMotor = new CANSparkMax(Constants.Climbing.rightMotorId, MotorType.kBrushless);

    for (CANSparkBase motor : Arrays.asList(m_leftClimbMotor, m_rightClimbMotor)) {
      motor.setIdleMode(IdleMode.kBrake);
      motor.burnFlash();
    }
  }

  private void setSpeed(double speed) {
    for (CANSparkBase motor : Arrays.asList(m_leftClimbMotor, m_rightClimbMotor)) {
      motor.set(speed);
    }
  }

  public void startClimb() {
    setSpeed(Constants.Climbing.climbingMaxSpeeed);
  }

  public void startUnclimb() {
    setSpeed(-Constants.Climbing.climbingMaxSpeeed);
  }

  public void stopClimb() {
    setSpeed(0);
  }
}
