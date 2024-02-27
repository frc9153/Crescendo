// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.UpDownCommand;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.Esophagus;
import frc.robot.subsystems.NetworktableReader;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.utils.Vector2;

public class RobotContainer {
    Joystick m_driverJoystick = new Joystick(Constants.HID.driverJoystickPort); // That logitech scary airplane looking thing
    DriveSwerve m_driveSwerve = new DriveSwerve();
    Esophagus m_esophagus = new Esophagus(); // Green-wheeled feeding machine
    UpAndDownForever m_upDown = new UpAndDownForever(); // Shoulder type thing
    Archerfish m_archerfish = new Archerfish(); // Fast spinning shootey bit
    NetworktableReader m_networkTableReader = new NetworktableReader(m_driveSwerve);

    public RobotContainer() {
        yankEveryCameraFeedWeCanLikeThatFatGuyFromYakuzaThatLivesUnderTheRiver();
        configureBindings();
    }

    private void yankEveryCameraFeedWeCanLikeThatFatGuyFromYakuzaThatLivesUnderTheRiver() {
        // If it thinks we have 10 cameras something is very wrong or we got rich
        // TODO: If rich, update accordingly
        for (int i=0; i<10; i++) {
            try {
                CameraServer.startAutomaticCapture(i);
            } catch (Exception e) {
                break;
            }
        }
    }

    private void configureBindings() {
        JoystickButton resetSwerveHeadingButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.resetSwerveHeadingButton);
        resetSwerveHeadingButton.onTrue(new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));

        JoystickButton armStoreButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.armStoreButton);
        armStoreButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.STORE));

        JoystickButton esophagusFeedButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.esophagusFeedButton);
        esophagusFeedButton.onTrue(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus));
        esophagusFeedButton.onFalse(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus));

        JoystickButton fireButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.archerfishFireButton);
        fireButton.onTrue(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish));
        fireButton.onFalse(new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish));

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
