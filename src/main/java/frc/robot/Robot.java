// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
    // TO DEAREST G: Pls don't put robo logic here! - Much love, G
    // TO DEAREST G: put sooooo much robo logic here hehehehehehhehehehhee - Much love, J

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        m_chooser.setDefaultOption("Do NOTHING!", m_robotContainer.m_doNothing);
        // m_chooser.addOption("Test", m_robotContainer.m_test);

        m_chooser.addOption("Shoot; Shoot; Shoot; Shoot", m_robotContainer.m_ShootShootShootShoot);
        m_chooser.addOption("Shoot; Shoot; Shoot; Piece", m_robotContainer.m_ShootShootShootPiece);
        m_chooser.addOption("Shoot; Shoot; Right Shoot", m_robotContainer.m_ShootShootRightShoot);
        m_chooser.addOption("Shoot; Shoot; Right Piece", m_robotContainer.m_ShootShootRightPiece);
        m_chooser.addOption("Shoot; Shoot; Left Shoot", m_robotContainer.m_ShootShootLeftShoot);
        m_chooser.addOption("Shoot; Shoot; Left Piece", m_robotContainer.m_ShootShootLeftPiece);
        m_chooser.addOption("Shoot; Shoot", m_robotContainer.m_ShootShoot);
        m_chooser.addOption("Shoot; Piece", m_robotContainer.m_ShootPiece);
        m_chooser.addOption("Shoot; Do Nothing", m_robotContainer.m_ShootDoNothing);

        // Someday, maybe: Right Side Shoot; Rush; Piece
        m_chooser.addOption("Right Side Shoot; Rush", m_robotContainer.m_RightSideShootRush);
        m_chooser.addOption("Right Side Shoot; Shoot", m_robotContainer.m_RightSideShootShoot);
        m_chooser.addOption("Right Side Shoot; Piece", m_robotContainer.m_RightSideShootPiece);
        m_chooser.addOption("Right Side Shoot; Mobility", m_robotContainer.m_RightSideShootMobility);
        m_chooser.addOption("Right Side Shoot; Correct", m_robotContainer.m_RightSideShootCorrect);
        
        // Someday, maybe: Left Side Shoot; Rush; Piece
        m_chooser.addOption("Left Side Shoot; Rush", m_robotContainer.m_LeftSideShootRush);
        m_chooser.addOption("Left Side Shoot; Shoot", m_robotContainer.m_LeftSideShootShoot);
        m_chooser.addOption("Left Side Shoot; Piece", m_robotContainer.m_LeftSideShootPiece);
        m_chooser.addOption("Left Side Shoot; Mobility", m_robotContainer.m_LeftSideShootMobility);
        m_chooser.addOption("Left Side Shoot; Correct", m_robotContainer.m_LeftSideShootCorrect);

        m_chooser.addOption("(Red) Amp; Rush", m_robotContainer.m_RedAmpRush);
        m_chooser.addOption("(Red) Amp; Left Shoot", m_robotContainer.m_RedAmpShoot);
        m_chooser.addOption("(Red) Amp; Piece", m_robotContainer.m_RedAmpPiece);
        m_chooser.addOption("(Red) Amp; Do Nothing", m_robotContainer.m_RedAmpDoNothing);

        m_chooser.addOption("(Blue) Amp; Rush", m_robotContainer.m_BlueAmpRush);
        m_chooser.addOption("(Blue) Amp; Right Shoot", m_robotContainer.m_BlueAmpShoot);
        m_chooser.addOption("(Blue) Amp; Piece", m_robotContainer.m_BlueAmpPiece);
        m_chooser.addOption("(Blue) Amp; Do Nothing", m_robotContainer.m_BlueAmpDoNothing);


        SmartDashboard.putData("Autonomous Routine", m_chooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_chooser.getSelected();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
