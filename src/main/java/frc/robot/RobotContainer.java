// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.UpDownCommand;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.Esophagus;
import frc.robot.subsystems.NetworktableReader;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Vector2;

public class RobotContainer {
    Joystick m_driverJoystick = new Joystick(Constants.HID.driverJoystickPort); // That logitech scary airplane looking thing
    XboxController m_operatorController = new XboxController(Constants.HID.operatorJoystickPort);
    DriveSwerve m_driveSwerve = new DriveSwerve();
    Esophagus m_esophagus = new Esophagus(); // Green-wheeled feeding machine
    UpAndDownForever m_upDown = new UpAndDownForever(); // Shoulder type thing
    Archerfish m_archerfish = new Archerfish(); // Fast spinning shootey bit
    Climbing m_climber = new Climbing(); // Fast spinning shootey bit
    NetworktableReader m_networkTableReader = new NetworktableReader(m_driveSwerve);

    public RobotContainer() {
        System.out.println("HELLO ; We are starting");
        // yankEveryCameraFeedWeCanLikeThatFatGuyFromYakuzaThatLivesUnderTheRiver();
        configureBindings();
    }

    private void yankEveryCameraFeedWeCanLikeThatFatGuyFromYakuzaThatLivesUnderTheRiver() {
        // If it thinks we have 10 cameras something is very wrong or we got rich
        // TODO: If rich, update accordingly
        System.out.println("Hunting");
        for (int i=0; i<10; i++) {
            try {
                CameraServer.startAutomaticCapture(i);
            } catch (Exception e) {
                break;
            }
        }
    }

    private void drive(Vector2 speed, double rotSpeed, boolean isFieldRelative) {
        // Hijack swerve to align when requested

        // TODO: Setup limelight
        if (false) {
            // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-aiming-and-ranging
            double kP = .035;

            // uniform
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
            targetingAngularVelocity *= Constants.Drive.maxRotSpeedRadsPerSec;

            // invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;
            rotSpeed = targetingAngularVelocity;

            isFieldRelative = false;
        }

        m_driveSwerve.drive(speed, rotSpeed, isFieldRelative);
    }

    private void configureBindings() {
        JoystickButton resetSwerveHeadingButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.resetSwerveHeadingButton);
        resetSwerveHeadingButton.onTrue(new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));

        JoystickButton armIntakeButton = new JoystickButton(m_operatorController, Constants.HID.Binds.armIntakeButton);
        armIntakeButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE));
        JoystickButton armShootButton = new JoystickButton(m_operatorController, Constants.HID.Binds.armShootButton);
        armShootButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT));
        JoystickButton armStoreButton = new JoystickButton(m_operatorController, Constants.HID.Binds.armStoreButton);
        armStoreButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.STORE));
        JoystickButton armAmpButton = new JoystickButton(m_operatorController, Constants.HID.Binds.armAmpButton);
        armAmpButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP));

        JoystickButton esophagusFeedButton = new JoystickButton(m_operatorController, Constants.HID.Binds.esophagusFeedButton);
        esophagusFeedButton.onTrue(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus));
        esophagusFeedButton.onFalse(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus));

        JoystickButton fireButton = new JoystickButton(m_operatorController, Constants.HID.Binds.archerfishFireButton);
        fireButton.onTrue(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish));
        fireButton.onFalse(new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish));

        JoystickButton climberPullButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.climberPullButton);
        climberPullButton.onTrue(new InstantCommand(() -> m_climber.startClimb(), m_climber));
        climberPullButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));

        JoystickButton climberPushButton = new JoystickButton(m_driverJoystick, Constants.HID.Binds.climberPushButton);
        climberPushButton.onTrue(new InstantCommand(() -> m_climber.startUnclimb(), m_climber));
        climberPushButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));

        m_driveSwerve.setDefaultCommand(
                // Inline command instantiation--will run a lot forever. Runs first lambda arg
                // as command code and locks following args as requirements
                new RunCommand(
                        () -> drive(
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
