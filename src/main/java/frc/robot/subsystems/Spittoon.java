package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spittoon extends SubsystemBase {
    // TODO: Brushless/brushed
    private CANSparkBase m_vomitMotor;
    private RelativeEncoder m_vomitEncoder;
    private SparkPIDController m_PIDController;

    public Spittoon() {
        m_vomitMotor = new CANSparkMax(Constants.Spittoon.spittoonId, MotorType.kBrushless);
        m_vomitEncoder = m_vomitMotor.getEncoder();
        // m_PIDController = m_vomitMotor.getPIDController();

        // m_PIDController.setP(Constants.Spittoon.spittoonP);
        // m_PIDController.setI(Constants.Spittoon.spittoonI);
        // m_PIDController.setD(Constants.Spittoon.spittoonD);
        // m_PIDController.setIZone(Constants.Spittoon.spittoonIZone);
        // m_PIDController.setFF(Constants.Spittoon.spittoonFF);
        // m_PIDController.setOutputRange(Constants.Spittoon.spittoonMinSpeed, Constants.Spittoon.spittoonMaxSpeed);

        m_vomitMotor.setIdleMode(IdleMode.kCoast);
        m_vomitMotor.burnFlash();
    }

    public void startFireSequence() {
        m_vomitMotor.set(0.5);
    }

    public void abortFireSequence() {
        m_vomitMotor.set(0.0);
    }
}
