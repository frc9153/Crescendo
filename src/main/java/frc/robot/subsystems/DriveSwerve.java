// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Vector2;

public class DriveSwerve extends SubsystemBase {
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        Constants.Drive.CAN.frontLeftThrustId,
        Constants.Drive.CAN.frontLeftSteerId,
        Constants.Drive.Chassis.frontLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        Constants.Drive.CAN.frontRightThrustId,
        Constants.Drive.CAN.frontRightSteerId,
        Constants.Drive.Chassis.frontRightChassisAngularOffset
    );

    private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
        Constants.Drive.CAN.backLeftThrustId,
        Constants.Drive.CAN.backLeftSteerId,
        Constants.Drive.Chassis.backLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_backRight = new MAXSwerveModule(
        Constants.Drive.CAN.backRightThrustId,
        Constants.Drive.CAN.backRightSteerId,
        Constants.Drive.Chassis.backRightChassisAngularOffset
    );

    /** Creates a new DriveSwerve. */
    public DriveSwerve() {
    }

    @Override
    public void periodic() {
        // TODO: Update odometry
    }

    /**
     * Actually do any driving. Called lots by whatever's driving the driver
     * 
     * @param speed           Uniform Vector2 representing robot speed. NOTE: x is
     *                        forward/backward here, y is sideways.
     * @param rotSpeed
     * @param isFieldRelative Determines if the given speed is global (i.e.
     *                        field-oriented) or local.
     */
    public void drive(Vector2 speed, double rotSpeed, boolean isFieldRelative) {
        // https://github.com/REVrobotics/MAXSwerve-Java-Template -- Copyright REV Robotics, BSD-3-Clause
        // TODO: rate limit "for smoother control". hope this doesnt explode
        // TODO: FIELD RELATIVE
        Vector2 speedDeliveredMetersPerSec = speed.mult_by(Constants.Drive.maxSpeedMetersPerSec);
        double rotDelivered = rotSpeed * Constants.Drive.maxRotSpeedRadsPerSec;

        var swerveModuleStates = Constants.Drive.Chassis.driveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(speedDeliveredMetersPerSec.x, speedDeliveredMetersPerSec.y, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.maxSpeedMetersPerSec);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }
}
