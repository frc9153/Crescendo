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
import frc.robot.commands.IntakeBackwardCommand;
import frc.robot.utils.Vector2;
import frc.robot.Constants;

public class CenterSideShoot extends SequentialCommandGroup {
    public double speed;
    public double dist_to_note;
    public double side_dist;

    public CenterSideShoot(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus, double right_or_left) {
        speed = Constants.Autonomous.autoSpeed;
        dist_to_note = (Constants.Autonomous.Speaker_Front.SpeakerToSpikeMark/speed);
        side_dist = (Constants.Autonomous.Speaker_Front.SpikeMarkToSpikeMark/speed);

        addCommands(
            new ParallelCommandGroup(
                new DriveCommand(m_driveSwerve, new Vector2(0, -speed*right_or_left), 0, true).withTimeout(side_dist*0.95),
                new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
                new SequentialCommandGroup(
                    new IntakeBackwardCommand(m_esophagus).withTimeout(0.1),
                    new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1))),
            new DriveCommand(m_driveSwerve, new Vector2(-speed, 0), 0, true).withTimeout(dist_to_note),
            new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.6)
        );
    }
}