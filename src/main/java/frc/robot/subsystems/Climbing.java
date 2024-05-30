// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbing extends SubsystemBase {
  // just like that swedish game "Climbing"
  private CANSparkBase m_leftClimbMotor;
  private CANSparkBase m_rightClimbMotor;
  private SparkLimitSwitch m_leftLimitSwitch;
  private SparkLimitSwitch m_rightLimitSwitch;


  public Climbing() {
    // i swear on my left foot that there was a class to control a pair of motors in tandem but its escaping me
    m_leftClimbMotor = new CANSparkMax(Constants.Climbing.leftMotorId, MotorType.kBrushless);
    m_rightClimbMotor = new CANSparkMax(Constants.Climbing.rightMotorId, MotorType.kBrushless);

    // m_leftLimitSwitch = m_leftClimbMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    // m_leftLimitSwitch.enableLimitSwitch(true);

    // m_rightLimitSwitch = m_leftClimbMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    // m_rightClimbMotor.follow(m_leftClimbMotor, false);

    for (CANSparkBase motor : Arrays.asList(m_leftClimbMotor, m_rightClimbMotor)) {
      motor.setIdleMode(IdleMode.kBrake);
      motor.burnFlash();
    }
  }

  private void setSpeed(double speed) {
    for (CANSparkBase motor : Arrays.asList(m_leftClimbMotor, m_rightClimbMotor)) {
      motor.set(speed);
    }
    // m_leftClimbMotor.set(speed);
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

  @Override
  public void periodic() {
    // if (m_rightLimitSwitch.isPressed()) m_rightClimbMotor.set(0.0);
  }
}
