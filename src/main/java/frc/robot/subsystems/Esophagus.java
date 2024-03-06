package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Esophagus extends SubsystemBase {
    private CANSparkBase m_raspberryMotor;

    public Esophagus() {
        m_raspberryMotor = new CANSparkMax(
            Constants.Esophagus.esophagusId,
            MotorType.kBrushless
        );
        m_raspberryMotor.setIdleMode(IdleMode.kCoast);
        m_raspberryMotor.setInverted(true);
        m_raspberryMotor.burnFlash();
    }

    public void startFeeding() {
        // make constants okayyy!?
        m_raspberryMotor.set(Constants.Esophagus.esophagusSpeed);
    }

    public void stopFeeding() {
        m_raspberryMotor.set(0.0);
    }
}
