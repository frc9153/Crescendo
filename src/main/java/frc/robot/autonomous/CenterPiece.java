// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSwerve;
import frc.robot.subsystems.UpAndDownForever;
import frc.robot.subsystems.Archerfish;
import frc.robot.subsystems.Esophagus;
import frc.robot.commands.WindThenScore;
import frc.robot.commands.UpDownCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

public class CenterPiece extends SequentialCommandGroup {
    public double speed;
    public double dist_to_note;

    public CenterPiece(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus) {
        speed = Constants.Autonomous.autoSpeed;
        dist_to_note = (Constants.Autonomous.Speaker_Front.SpeakerToSpikeMark/speed);

        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, true).withTimeout(dist_to_note)),
                new SequentialCommandGroup(
                    new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
                    new IntakeCommand(m_esophagus).withTimeout(Constants.Autonomous.IntakeGiveUp)))
        );
    }
}