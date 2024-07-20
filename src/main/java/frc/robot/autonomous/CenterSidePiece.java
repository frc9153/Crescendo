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

public class CenterSidePiece extends SequentialCommandGroup {
    public double speed;
    public double dist_to_note;
    public double side_dist;

    public CenterSidePiece(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus, double right_or_left) {
        speed = Constants.Autonomous.autoSpeed;
        dist_to_note = (Constants.Autonomous.Speaker_Front.SpeakerToSpikeMark/speed);
        side_dist = (Constants.Autonomous.Speaker_Front.SpikeMarkToSpikeMark/speed);

        addCommands(
            // new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, false).withTimeout(0.2),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new DriveCommand(m_driveSwerve, new Vector2(speed*0.325, speed*right_or_left), 0, true).withTimeout(side_dist*0.65*Math.sqrt(2)),
                    new WaitCommand(0.3),
                    new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, true).withTimeout(dist_to_note-(side_dist*0.5))),
                new SequentialCommandGroup(
                    new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
                    new IntakeCommand(m_esophagus).withTimeout(Constants.Autonomous.IntakeGiveUp+0.5)))
        );
    }
}