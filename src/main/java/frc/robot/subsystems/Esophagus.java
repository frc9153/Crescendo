package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Esophagus extends SubsystemBase {
    private CANSparkBase m_raspberryMotor;
    private BooleanSupplier m_canRun;
    private boolean m_run_forward = false;

    public Esophagus(BooleanSupplier canRun) {
        m_raspberryMotor = new CANSparkMax(
            Constants.Esophagus.esophagusId,
            MotorType.kBrushless
        );
        m_raspberryMotor.setIdleMode(IdleMode.kCoast);
        m_raspberryMotor.setInverted(true);
        m_raspberryMotor.burnFlash();
        m_canRun = canRun;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void startFeeding() {
        m_run_forward = true;
        m_raspberryMotor.set(Constants.Esophagus.esophagusSpeed);
    }

    public void startReverse() {
        m_run_forward = false;
        m_raspberryMotor.set(Constants.Esophagus.reverseSpeed);
    }

    public void stopFeeding() {
        m_run_forward = false;
        m_raspberryMotor.set(0.0);
    }

    @Override
    public void periodic() {
        if (!m_canRun.getAsBoolean() && m_run_forward) {
            double speed = Constants.Esophagus.esophagusSpeed / 3;
            m_raspberryMotor.set(speed);
        };
    }
}
