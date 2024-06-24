// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Esophagus;
import frc.robot.commands.WindThenScore;
import frc.robot.commands.UpDownCommand;
import frc.robot.Constants;

public class ShootDoNothing extends SequentialCommandGroup {

    public ShootDoNothing(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus) {
        addCommands(
            new ParallelCommandGroup(
                new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
                new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT)),
            new WindThenScore(m_archerfish, m_esophagus).withTimeout(1.5)
        );
    }
}