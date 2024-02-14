// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.utils.Vector2;

public class RobotContainer {
    // That logitech scary airplane looking thing
    Joystick m_driverJoystick = new Joystick(Constants.HID.driverJoystickPort);
    DriveSwerve m_driveSwerve = new DriveSwerve();

    public RobotContainer() {
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);
        configureBindings();
    }

    private void configureBindings() {
        JoystickButton resetEncoderButton = new JoystickButton(m_driverJoystick, 7);
        resetEncoderButton.onTrue(new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));

        m_driveSwerve.setDefaultCommand(
                // Inline command instantiation--will run a lot forever. Runs first lambda arg
                // as command code and locks following args as requirements
                new RunCommand(
                        () -> m_driveSwerve.drive(
                                new Vector2(
                                        -m_driverJoystick.getY(),
                                        -m_driverJoystick.getX()).deadband(Constants.HID.driverJoystickDeadband),
                                -m_driverJoystick.getTwist(),
                                !m_driverJoystick.getRawButton(1)),
                        m_driveSwerve));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
