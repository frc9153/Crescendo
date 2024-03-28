// https://github.com/REVrobotics/MAXSwerve-Java-Template
// Copyright REV Robotics, BSD-3-Clause

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Vector2;

public class NetworktableReader extends SubsystemBase {
    private double m_lastBadump = 0;
    private double m_lastBadumpToken = -0.11031;
    private boolean m_pause = false;
    private final DriveSwerve m_driveSwerve;
    private final DoubleArraySubscriber m_vectorSub;
    private final DoubleSubscriber m_badumpTokenSub;
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public NetworktableReader(DriveSwerve driveswerve) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("evil_manipulation");

        m_vectorSub = table.getDoubleArrayTopic("drive_vector").subscribe(new double[] { 0.0, 0.0 });
        m_badumpTokenSub = table.getDoubleTopic("badump_token").subscribe(m_lastBadumpToken);
        m_driveSwerve = driveswerve;

        m_lastBadump = System.currentTimeMillis();
    }

    @Override
    public void periodic() {
        Vector2 moveVector = Vector2.fromArray(m_vectorSub.get());
        double timeSinceBadump = System.currentTimeMillis() - m_lastBadump;
        double badump = m_badumpTokenSub.get();

        // SmartDashboard.putNumber("GYRO_X", gyro.getDisplacementX());
        // SmartDashboard.putNumber("GYRO_Y", gyro.getDisplacementY());
        // SmartDashboard.putNumber("GYRO_Z", gyro.getDisplacementZ());

        if (m_lastBadumpToken == m_lastBadump) {
            System.out.println("[fatal] STUPID TOKEN DOESN'T MATCH STUPID OVER TOKEN!!!!!!!!!!!111!!");
            m_pause = true;
        }

        if (m_pause) {
            // Wahh!
            return;
        }

        // TODO:
        // m_driveSwerve.drive(moveVector.normalized(), moveVector.magnitude(), true);
    }
}