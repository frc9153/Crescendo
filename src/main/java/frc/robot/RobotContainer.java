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
import frc.robot.autonomous.ShootDoNothing;
import frc.robot.autonomous.ShootPiece;
import frc.robot.autonomous.ShootShoot;
import frc.robot.autonomous.ShootShootPiece;
import frc.robot.autonomous.ShootShootShoot;
import frc.robot.autonomous.ShootShootShootPiece;
import frc.robot.autonomous.ShootShootShootShoot;
import frc.robot.autonomous.RightSideHeadingCorrect;
import frc.robot.autonomous.LeftSideHeadingCorrect;
import frc.robot.autonomous.RightSidePiece;
import frc.robot.autonomous.LeftSidePiece;
import frc.robot.autonomous.SideMobility;
import frc.robot.commands.AmpThenScore;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FancyDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.KidSafeWindThenScore;
import frc.robot.commands.ManualUpDownCommand;
import frc.robot.commands.UpDownCommand;
import frc.robot.commands.WindThenScore;
import frc.robot.commands.BooleanTriggeredManualUpDownCommand;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Climbing;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.Esophagus;
import frc.robot.subsystems.LaserCannon;
import frc.robot.subsystems.NetworktableReader;
import frc.robot.subsystems.TransformableGyro;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Vector2;

public class RobotContainer {
    CommandJoystick m_driverJoystick = new CommandJoystick(Constants.HID.driverJoystickPort); // That logitech scary
                                                                                              // airplane looking thing
    CommandXboxController m_operatorController = new CommandXboxController(Constants.HID.operatorJoystickPort);
    CommandJoystick m_childJoystick = new CommandJoystick(Constants.HID.childJoystickPort); // For kids at Peach Festival to
                                                                                            // prevent unscheduled disassembly
    TransformableGyro m_gyro = new TransformableGyro();
    DriveSwerve m_driveSwerve = new DriveSwerve(m_gyro);
    UpAndDownForever m_upDown = new UpAndDownForever(); // Shoulder type thing
    LaserCannon m_laserCannon = new LaserCannon(); // Intake Sensor
    Esophagus m_esophagus = new Esophagus(() -> m_upDown.getSetpoint() == Setpoint.INTAKE,
            () -> m_laserCannon.sensorTriggered()); // Green-wheeled feeding machine
    Archerfish m_archerfish = new Archerfish(); // Fast spinning shootey bit
    Climbing m_climber = new Climbing(); // Climbing
    NetworktableReader m_networkTableReader = new NetworktableReader(m_driveSwerve);
    boolean m_aiming = false;

    // Drive Command is Vector2(forward, side)
    public final Command m_doNothing = Commands.none();

    public final Command m_ShootDoNothing = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootPiece = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootShoot = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootShootPiece = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootShootShoot = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootShootShootPiece = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_ShootShootShootShoot = new SequentialCommandGroup(
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShootPiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new ShootShootShootShoot(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_RightSideShootCorrect = new SequentialCommandGroup(
        new RightSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_RightSideShootMobility = new SequentialCommandGroup(
        new RightSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new SideMobility(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_RightSideShootPiece = new SequentialCommandGroup(
        new RightSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new RightSidePiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_LeftSideShootCorrect = new SequentialCommandGroup(
        new LeftSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_LeftSideShootMobility = new SequentialCommandGroup(
        new LeftSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new SideMobility(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
    );
    public final Command m_LeftSideShootPiece = new SequentialCommandGroup(
        new LeftSideHeadingCorrect(m_gyro),
        new ShootDoNothing(m_driveSwerve, m_upDown, m_archerfish, m_esophagus),
        new LeftSidePiece(m_driveSwerve, m_upDown, m_archerfish, m_esophagus)
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
        for (int i = 0; i < 10; i++) {
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
            // double kP = 0.035;
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
        armAmpButton.whileTrue(new AmpThenScore(m_upDown, m_archerfish, m_esophagus));

        Trigger esophagusOutButton = m_operatorController.button(Constants.HID.Binds.Operator.intakeReverseButton);
        esophagusOutButton.onTrue(new InstantCommand(() -> m_esophagus.startReverse(), m_esophagus));
        esophagusOutButton.onFalse(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus));

        Trigger esophagusFeedButton = m_operatorController.leftTrigger(0.3);
        esophagusFeedButton.whileTrue(new IntakeCommand(m_esophagus));

        Trigger fireSlowButton = m_operatorController.button(Constants.HID.Binds.Operator.shooterSlowButton);
        fireSlowButton.onTrue(new InstantCommand(() -> m_archerfish.slowSpin(), m_archerfish));
        fireSlowButton.onFalse(new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish));

        Trigger fireButton = m_operatorController.rightTrigger(0.3);
        fireButton.whileTrue(new WindThenScore(m_archerfish, m_esophagus));

        Trigger climberPullButton = m_operatorController.pov(0);
        Trigger climberPushButton = m_operatorController.pov(180);
        climberPullButton.onTrue(new InstantCommand(() -> m_climber.startClimb(), m_climber));
        climberPullButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));
        climberPushButton.onTrue(new InstantCommand(() -> m_climber.startUnclimb(), m_climber));
        climberPushButton.onFalse(new InstantCommand(() -> m_climber.stopClimb(), m_climber));

        // Double check this with the driver station before running anything
        Trigger manualArmTrigger = m_operatorController.button(Constants.HID.Operator.manualArmTriggerButton);
        manualArmTrigger.whileTrue(new BooleanTriggeredManualUpDownCommand(m_upDown, () -> -m_operatorController.getX(), () -> manualArmTrigger));

        Trigger magicAimButton = m_driverJoystick.button(Constants.HID.Binds.Driver.magicAimButton);
        magicAimButton.onTrue(new InstantCommand(() -> m_aiming = true));
        magicAimButton.onFalse(new InstantCommand(() -> m_aiming = false));

        // Twist modified for Peach Festival compatibility
        m_driveSwerve.setDefaultCommand(
                // Inline command instantiation--will run a lot forever. Runs first lambda arg
                // as command code and locks following args as requirements
                new RunCommand(
                        () -> drive(
                                new Vector2(-m_driverJoystick.getY(), -m_driverJoystick.getX())
                                        .deadband(Constants.HID.driverJoystickDeadband)
                                        .multBy(
                                                1.0 - (m_driverJoystick.getThrottle() + 1.0) / 2.0),
                                (Math.abs(m_childJoystick.getTwist()) > Math.abs(m_driverJoystick.getTwist())) ? -(m_childJoystick.getTwist()*0.5) : -m_driverJoystick.getTwist(),
                                !m_driverJoystick.button(Constants.HID.Binds.Driver.robotOrientedDriveButton)
                                        .getAsBoolean()),
                        m_driveSwerve));
        

        // Peach Festival disaster averting alternate safety controls

        Trigger childIntakeButton = m_childJoystick.button(Constants.HID.Binds.Child.intakeButton);
        childIntakeButton.whileTrue(new IntakeCommand(m_esophagus));

        Trigger childShootButton = m_childJoystick.button(Constants.HID.Binds.Child.shootButton);
        childShootButton.whileTrue(new KidSafeWindThenScore(m_archerfish, m_esophagus));

        Trigger childManualArmUp = m_childJoystick.povUp();
        Trigger childManualArmDown = m_childJoystick.povDown();
        childManualArmUp.whileTrue(new ManualUpDownCommand(m_upDown, () -> -Constants.UpDownForever.manualSpeed));
        childManualArmDown.whileTrue(new ManualUpDownCommand(m_upDown, () -> Constants.UpDownForever.manualSpeed));
    }
}
