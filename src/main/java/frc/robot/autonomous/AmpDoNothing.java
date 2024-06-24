// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Esophagus;
import frc.robot.commands.WindThenScore;
import frc.robot.commands.UpDownCommand;
import frc.robot.commands.AmpThenScore;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

public class AmpDoNothing extends SequentialCommandGroup {
    public double speed;
    public double intial_dist;
    public double wall_dist;

    public AmpDoNothing(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus, double red_or_blue) {
        speed = Constants.Autonomous.autoSpeed;
        intial_dist = (Constants.Autonomous.Amp.InitialAlign/speed);
        wall_dist = (Constants.Autonomous.Amp.InitialToWall/speed);
        System.out.println("CEDRIC FIX THIS NOW!!!!!");

        addCommands(
            // new InstantCommand(() -> m_gyro.fakeReset(90.0*red_or_blue), m_gyro),
            new DriveCommand(m_driveSwerve, new Vector2(-speed, 0), 0, false).withTimeout(wall_dist),
            new ParallelCommandGroup(
                new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(0.1),
                new DriveCommand(m_driveSwerve, new Vector2(0, -speed*red_or_blue), 0, false).withTimeout(intial_dist)),
            new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(0.5)
        );
    }
}