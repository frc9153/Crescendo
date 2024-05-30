// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Archerfish extends SubsystemBase {
    private CANSparkBase m_fireMotor;
    private SparkPIDController m_fireMotorPID;
    private RelativeEncoder m_fireMotorEncoder;
    private SlewRateLimiter m_filter = new SlewRateLimiter(0.65, -0.23, 0.0);
    private double m_targetSpeed = 0.0;
    private SlewRateLimiter m_velocityLimiter = new SlewRateLimiter(6000, -2700, 0.0);
    private double m_targetVelocity = 0.0;


    public Archerfish() {
        m_fireMotor = new CANSparkMax(
                Constants.Archerfish.archerfishId,
                MotorType.kBrushless
        );
        m_fireMotor.setIdleMode(IdleMode.kCoast);
        m_fireMotor.setInverted(true);
        m_fireMotor.burnFlash();

        m_fireMotorEncoder = m_fireMotor.getEncoder();

        m_fireMotorPID = m_fireMotor.getPIDController();
        // REVLibError e1 = m_fireMotorPID.setP(Constants.Archerfish.archerfishP);//kHALerror
        // REVLibError e2 = m_fireMotorPID.setI(Constants.Archerfish.archerfishI);//kHALerror
        // REVLibError e3 = m_fireMotorPID.setD(Constants.Archerfish.archerfishD);//kHALerror
        // REVLibError e4 = m_fireMotorPID.setIZone(Constants.Archerfish.archerfishIZone);//kHALerror
        // REVLibError e5 = m_fireMotorPID.setFF(Constants.Archerfish.archerfishFF);
        // REVLibError e6 = m_fireMotorPID.setOutputRange(
        //         -Constants.Archerfish.archerfishMaxSpeed,
        //         Constants.Archerfish.archerfishMaxSpeed);
        REVLibError e7 = m_fireMotorPID.setFeedbackDevice(m_fireMotorEncoder);

        // System.out.println("EVIL PREEXISTING?:");
        // System.out.println(e1);
        // System.out.println(e2);
        // System.out.println(e3);
        // System.out.println(e4);
        // System.out.println(e5);
        // System.out.println(e6);
        // System.out.println(e7);

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    private void setSpeed(double speed) {
        m_targetSpeed = speed;
    }

    public void setVelocity(double velocity) {
        m_targetVelocity = velocity;
    }

    public void jerkToSpin() {
        setVelocity(Constants.Archerfish.archerfishVelocity);
        m_velocityLimiter.reset(m_targetVelocity);
    }

    public void startSpin() {
        // setSpeed(Constants.Archerfish.archerfishSpeed);
        setVelocity(Constants.Archerfish.archerfishVelocity);
    }

    public void stopSpin() {
        // setSpeed(0.0);
        setVelocity(0.0);
    }

    public void slowSpin() {
        // setSpeed(Constants.Archerfish.archerfishSpeedSlow);
        setVelocity(Constants.Archerfish.archerfishVelocitySlow);
    }

    public void childSpin() {
        // setSpeed(Constants.Archerfish.archerfishSpeedChild);
        setVelocity(Constants.Archerfish.archerfishVelocityChild);
    }

    public boolean isAtSpeed() {
        // if (m_filter.calculate(m_targetSpeed) == m_targetSpeed) {
        //     return true;
        // }
        if (Math.abs(m_fireMotorEncoder.getVelocity() - m_targetVelocity) < Constants.Archerfish.archerfishPIDEpsilon) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {

        // m_fireMotor.set(m_filter.calculate(m_targetSpeed));
        //System.out.println(m_fireMotorEncoder.getVelocity());

        m_fireMotorPID.setReference(m_velocityLimiter.calculate(m_targetVelocity), ControlType.kVelocity);

        SmartDashboard.putNumber("Archerfish Speed", m_fireMotorEncoder.getVelocity());
        SmartDashboard.putNumber("Archerfish Target", m_targetVelocity);
        SmartDashboard.putNumber("Archerfish Applied Voltage", m_fireMotor.getAppliedOutput());
    }
}
