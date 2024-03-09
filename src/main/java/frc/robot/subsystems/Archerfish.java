// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Archerfish extends SubsystemBase {
    private CANSparkBase m_fireMotor;
    private SlewRateLimiter m_filter = new SlewRateLimiter(0.2);
    private double m_targetSpeed = 0.0;


    public Archerfish() {
        m_fireMotor = new CANSparkMax(
                Constants.Archerfish.archerfishId,
                MotorType.kBrushless
        );
        m_fireMotor.setIdleMode(IdleMode.kCoast);
        m_fireMotor.setInverted(true);
        m_fireMotor.burnFlash();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    private void setSpeed(double speed) {
        // m_targetSpeed = speed;
        m_fireMotor.set(speed);
    }

    public void startSpin() {
        setSpeed(Constants.Archerfish.archerfishSpeed);
    }

    public void stopSpin() {
        setSpeed(0.0);
    }

    @Override
    public void periodic() {
        //m_fireMotor.set(m_filter.calculate(m_targetSpeed));
        //m_fireMotor.set(m_targetSpeed);
    }
}
