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
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

public class SidePiece extends SequentialCommandGroup {
    public double speed;
    public double slant_dist;
    public double spike_dist;

    public SidePiece(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus, double right_or_left) {
        speed = Constants.Autonomous.autoSpeed;
        slant_dist = (Constants.Autonomous.Speaker_Side.SidePieceSlant/speed);
        spike_dist = (Constants.Autonomous.Speaker_Side.SidePieceSpikeMark/speed);

        addCommands(
            new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, false).withTimeout(slant_dist),
            new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5*right_or_left, false).withTimeout(Constants.Autonomous.Turn60),
            new ParallelCommandGroup(
                new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, false).withTimeout(spike_dist),
                new SequentialCommandGroup(
                    new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
                    new IntakeCommand(m_esophagus).withTimeout(Constants.Autonomous.IntakeGiveUp)))
        );
    }
}