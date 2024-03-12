// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.UpDownForever.Setpoint;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FancyDriveCommand;
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
    CommandJoystick m_driverJoystick = new CommandJoystick(Constants.HID.driverJoystickPort); // That logitech scary airplane looking thing
    CommandXboxController m_operatorController = new CommandXboxController(Constants.HID.operatorJoystickPort);
    DriveSwerve m_driveSwerve = new DriveSwerve();
    UpAndDownForever m_upDown = new UpAndDownForever(); // Shoulder type thing
    Esophagus m_esophagus = new Esophagus(() -> m_upDown.getSetpoint() == Setpoint.INTAKE); // Green-wheeled feeding machine
    Archerfish m_archerfish = new Archerfish(); // Fast spinning shootey bit
    Climbing m_climber = new Climbing(); // Climbing
    NetworktableReader m_networkTableReader = new NetworktableReader(m_driveSwerve);
    boolean m_aiming = false;

    // Drive Command is Vector2(forward, side)
    public final Command m_doNothing = Commands.none();
    public final Command m_test = new SequentialCommandGroup(
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
        new FancyDriveCommand(m_driveSwerve, new Vector2(0.35, 0), 0, true, 1, 0)
        );
    public final Command m_AmpPiece = new SequentialCommandGroup(
        new DriveCommand(m_driveSwerve, new Vector2(0, -0.5), 0, false).withTimeout(0.7),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish),
            new WaitCommand(0.7)),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish),
            new DriveCommand(m_driveSwerve, new Vector2(0, 0.5), 0, false).withTimeout(0.2)),
        new DriveCommand(m_driveSwerve, new Vector2(0.5, 0), 0, false).withTimeout(0.7),
        new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(1.2),
        new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
        new WaitCommand(0.3),
        new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
        new DriveCommand(m_driveSwerve, new Vector2(0.5, 0), 0, false).withTimeout(1.5),
        new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus)
    );
    public final Command m_AmpShoot = new SequentialCommandGroup(
        new DriveCommand(m_driveSwerve, new Vector2(0, -0.5), 0, false).withTimeout(0.7),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
        new WaitCommand(0.1),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish),
            new WaitCommand(0.7)),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish),
            new DriveCommand(m_driveSwerve, new Vector2(0, 0.5), 0, false).withTimeout(0.2)),
        new DriveCommand(m_driveSwerve, new Vector2(0.5, 0), 0, false).withTimeout(0.6),
        new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(1.2),
        new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
        new WaitCommand(0.3),
        new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
        new DriveCommand(m_driveSwerve, new Vector2(0.5, 0), 0, false).withTimeout(1.5),
        new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
        new ParallelCommandGroup(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
            new InstantCommand(() -> m_esophagus.startReverse(), m_esophagus),
            new WaitCommand(0.2)),
        new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
        new DriveCommand(m_driveSwerve, new Vector2(-0.5, 0), 0, false).withTimeout(1.3),
        new DriveCommand(m_driveSwerve, new Vector2(0, 0.5), 0, false).withTimeout(0.8),
        new DriveCommand(m_driveSwerve, new Vector2(-0.5, 0), 0, false).withTimeout(0.5),
        new ParallelCommandGroup(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish),
            new WaitCommand(2.0)),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
            new WaitCommand(0.7)),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish))
    );
    public final Command m_basicShoot = new SequentialCommandGroup(
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
        new WaitCommand(0.7),
        //new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
        //new WaitCommand(0.7),
        new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
        new ParallelCommandGroup(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish),
            new WaitCommand(2.0)),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
            new WaitCommand(0.7)),
        new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
            new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish),
            new DriveCommand(m_driveSwerve, new Vector2(0.5, 0), 0, false).withTimeout(1.0)),
        new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(1.0)
    );

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
        if (m_aiming) {
            // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-aiming-and-ranging
            //double kP = 0.035;
            double kP = 0.005;

            // uniform
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
            SmartDashboard.putNumber("target angular vel", targetingAngularVelocity);
            targetingAngularVelocity *= Constants.Drive.maxRotSpeedRadsPerSec;

            // invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;
            rotSpeed = targetingAngularVelocity;

            isFieldRelative = false;
        }

        m_driveSwerve.drive(speed, rotSpeed, isFieldRelative);
    }

    private void configureBindings() {
        Trigger resetSwerveHeadingButton = m_driverJoystick.button(Constants.HID.Binds.Driver.resetSwerveHeadingButton);
        resetSwerveHeadingButton.onTrue(new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));

        Trigger armIntakeButton = m_operatorController.button(Constants.HID.Binds.Operator.armIntakeButton);
        Trigger armShootButton = m_operatorController.button(Constants.HID.Binds.Operator.armShootButton);
        Trigger armStartButton = m_operatorController.button(Constants.HID.Binds.Operator.armStartButton);
        Trigger armAmpButton = m_operatorController.button(Constants.HID.Binds.Operator.armAmpButton);
        armIntakeButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE));
        armShootButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT));
        armStartButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.START));
        armAmpButton.onTrue(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP));

        Trigger esophagusOutButton = m_operatorController.button(Constants.HID.Binds.Operator.shooterReverseButton);
        esophagusOutButton.onTrue(new InstantCommand(() -> m_esophagus.startReverse(), m_esophagus));
        esophagusOutButton.onFalse(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus));

        Trigger esophagusFeedButton = m_operatorController.leftTrigger(0.3);
        esophagusFeedButton.onTrue(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus));
        esophagusFeedButton.onFalse(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus));

        Trigger fireButton = m_operatorController.rightTrigger(0.3);
        fireButton.onTrue(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish));
        fireButton.onFalse(new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish));

        Trigger climberPullButton = m_driverJoystick.pov(180);
        Trigger climberPushButton = m_driverJoystick.pov(0);
        climberPullButton.onTrue(new InstantCommand(() -> m_climber.startClimb(), m_climber));
        climberPullButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));
        climberPushButton.onTrue(new InstantCommand(() -> m_climber.startUnclimb(), m_climber));
        climberPushButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));

        Trigger magicAimButton = m_driverJoystick.button(Constants.HID.Binds.Driver.magicAimButton);
        magicAimButton.onTrue(new InstantCommand(() -> m_aiming = true));
        magicAimButton.onFalse(new InstantCommand(() -> m_aiming = false));

        m_driveSwerve.setDefaultCommand(
                // Inline command instantiation--will run a lot forever. Runs first lambda arg
                // as command code and locks following args as requirements
                new RunCommand(
                        () -> drive(
                                new Vector2(-m_driverJoystick.getY(), -m_driverJoystick.getX())
                                .deadband(Constants.HID.driverJoystickDeadband)
                                .mult_by(
                                    1.0 - (m_driverJoystick.getThrottle() + 1.0) / 2.0
                                ),
                                -m_driverJoystick.getTwist(),
                                !m_driverJoystick.button(Constants.HID.Binds.Driver.robotOrientedDriveButton).getAsBoolean()),
                        m_driveSwerve));
    }
}
