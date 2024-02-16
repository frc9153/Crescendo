package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spittoon extends SubsystemBase {
    // TODO: Brushless/brushed
    private CANSparkBase m_vomitMotor;
    private RelativeEncoder m_vomitEncoder = m_vomitMotor.getEncoder();

    public Spittoon() {
        m_vomitMotor = new CANSparkMax(Constants.Spittoon.spittoonId, MotorType.kBrushless);
        m_vomitEncoder = m_vomitMotor.getEncoder();
    }
}
