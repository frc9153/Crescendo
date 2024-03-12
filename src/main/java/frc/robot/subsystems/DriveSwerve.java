// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MAXSwerveModule;
import frc.robot.utils.Vector2;

public class DriveSwerve extends SubsystemBase {
        private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
                        Constants.Drive.CAN.frontLeftThrustId,
                        Constants.Drive.CAN.frontLeftSteerId,
                        Constants.Drive.Chassis.frontLeftChassisAngularOffset);

        private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
                        Constants.Drive.CAN.frontRightThrustId,
                        Constants.Drive.CAN.frontRightSteerId,
                        Constants.Drive.Chassis.frontRightChassisAngularOffset);

        private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
                        Constants.Drive.CAN.backLeftThrustId,
                        Constants.Drive.CAN.backLeftSteerId,
                        Constants.Drive.Chassis.backLeftChassisAngularOffset);

        private final MAXSwerveModule m_backRight = new MAXSwerveModule(
                        Constants.Drive.CAN.backRightThrustId,
                        Constants.Drive.CAN.backRightSteerId,
                        Constants.Drive.Chassis.backRightChassisAngularOffset);

        private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

        // Odometry class for tracking robot pose
        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
                        Constants.Drive.Chassis.driveKinematics,
                        Rotation2d.fromDegrees(m_gyro.getAngle()),
                        new SwerveModulePosition[] {
                                        m_frontLeft.getPosition(),
                                        m_frontRight.getPosition(),
                                        m_backLeft.getPosition(),
                                        m_backRight.getPosition()
                        });

        /** Creates a new DriveSwerve. */
        public DriveSwerve() {
        }

        @Override
        public void periodic() {
                // Update the odometry in the periodic block
                m_odometry.update(
                                Rotation2d.fromDegrees(m_gyro.getAngle()),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getPosition(),
                                                m_frontRight.getPosition(),
                                                m_backLeft.getPosition(),
                                                m_backRight.getPosition()
                                });
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(
                                Rotation2d.fromDegrees(m_gyro.getAngle()),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getPosition(),
                                                m_frontRight.getPosition(),
                                                m_backLeft.getPosition(),
                                                m_backRight.getPosition()
                                },
                                pose);
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
                // https://github.com/REVrobotics/MAXSwerve-Java-Template -- Copyright REV
                // Robotics, BSD-3-Clause
                // TODO: rate limit "for smoother control". hope this doesnt explode
                Vector2 speedDeliveredMetersPerSec = speed.multBy(Constants.Drive.maxSpeedMetersPerSec);
                double rotDelivered = rotSpeed * Constants.Drive.maxRotSpeedRadsPerSec;

                double angle = m_gyro.getAngle();

                SmartDashboard.putNumber("Gyro Angle", angle);

                // NOTE: Angle inverted here!!!!!!!!!
                var swerveModuleStates = Constants.Drive.Chassis.driveKinematics.toSwerveModuleStates(
                                isFieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                speedDeliveredMetersPerSec.x,
                                                                speedDeliveredMetersPerSec.y,
                                                                rotDelivered,
                                                                Rotation2d.fromDegrees(
                                                                                -angle))
                                                : new ChassisSpeeds(speedDeliveredMetersPerSec.x,
                                                                speedDeliveredMetersPerSec.y, rotDelivered));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.maxSpeedMetersPerSec);

                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);
        }

        public void zeroHeading() {
                System.out.println("Reset gyro!");
                m_gyro.reset();
        }

}
