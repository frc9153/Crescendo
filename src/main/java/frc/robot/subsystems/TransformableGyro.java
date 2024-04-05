package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransformableGyro extends SubsystemBase {
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private double m_rotationOffset = 0.0;

    public TransformableGyro() {

    }

    public double getAngle() {
        return (m_gyro.getAngle()-m_rotationOffset+360) % 360;
    }

    public void reset() {
        m_gyro.reset();
        m_rotationOffset = 0.0;
    }

    public void fakeReset(double angle) {
        m_gyro.reset();
        m_rotationOffset = angle;
    }
}
