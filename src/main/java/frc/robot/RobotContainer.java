// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.utils.Vector2;
import frc.robot.subsystems.DriveSwerve;

public class RobotContainer {
    // That logitech scary airplane looking thing
    Joystick m_driverJoystick = new Joystick(Constants.HID.driverJoystickPort);
    DriveSwerve m_driveSwerve = new DriveSwerve();

    public RobotContainer() {
        configureBindings();

        m_driveSwerve.setDefaultCommand(
            // Inline command instantiation--will run a lot forever. Runs first lambda arg
            // as command code and locks following args as requirements
            new RunCommand(
                () -> m_driveSwerve.drive(
                    new Vector2(
                        -m_driverJoystick.getY(),
                        -m_driverJoystick.getX()
                    ).deadband(Constants.HID.driverJoystickDeadband),
                    -m_driverJoystick.getTwist(),
                    false
                ),
                m_driveSwerve
            )
        );
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
