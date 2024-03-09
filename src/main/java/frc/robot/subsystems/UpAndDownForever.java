// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UpDownForever.Setpoint;

public class UpAndDownForever extends SubsystemBase {
    private Setpoint m_setpoint = Setpoint.START;
    private final CANSparkMax m_motor;
    private final SparkPIDController m_PIDController;
    private final SparkAbsoluteEncoder m_encoder;

    public UpAndDownForever() {
        m_motor = new CANSparkMax(Constants.UpDownForever.upDownMotorId, MotorType.kBrushless);

        m_PIDController = m_motor.getPIDController();
        m_encoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // PID Config setup
        m_PIDController.setP(Constants.UpDownForever.upDownP);
        m_PIDController.setI(Constants.UpDownForever.upDownI);
        m_PIDController.setD(Constants.UpDownForever.upDownD);
        m_PIDController.setIZone(Constants.UpDownForever.upDownIZone);
        m_PIDController.setFF(Constants.UpDownForever.upDownFF);
        m_PIDController.setOutputRange(
                -Constants.UpDownForever.upDownMaxSpeed,
                Constants.UpDownForever.upDownMaxSpeed);
        m_PIDController.setFeedbackDevice(m_encoder);

        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.burnFlash();
    }

    public Setpoint getSetpoint() {
        return m_setpoint;
    }

    public void killItNow() {
        m_setpoint = Constants.UpDownForever.Setpoint.INTAKE;
        m_motor.stopMotor();
    }

    public void gotoSetpoint(Setpoint setpoint) {
        m_setpoint = setpoint;
        m_PIDController.setReference(setpoint.targetPosition(), ControlType.kPosition);
    }

    public boolean isAtSetpoint() {
        return Math
                .abs(m_setpoint.targetPosition() - m_encoder.getPosition()) <= Constants.UpDownForever.upDownPIDEpsilon;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("updown encoder", m_encoder.getPosition());
    }
}
