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

public class ShootShootShootPiece extends SequentialCommandGroup {
    public double speed;
    public double dist_to_note;
    public double side_dist;

    public ShootShootShootPiece(DriveSwerve m_driveSwerve, UpAndDownForever m_upDown, Archerfish m_archerfish, Esophagus m_esophagus) {
        speed = Constants.Autonomous.autoSpeed;
        dist_to_note = (Constants.Autonomous.Speaker_Front.SpeakerToSpikeMark/speed);
        side_dist = (Constants.Autonomous.Speaker_Front.SpikeMarkToSpikeMark/speed);

        addCommands(
            new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, false).withTimeout(dist_to_note/2),
            new ParallelCommandGroup(
                new DriveCommand(m_driveSwerve, new Vector2(0, -speed), 0, false).withTimeout(side_dist),
                new SequentialCommandGroup(
                    new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE))),
            new ParallelCommandGroup(
                new DriveCommand(m_driveSwerve, new Vector2(speed, 0), 0, false).withTimeout(dist_to_note/2),
                new IntakeCommand(m_esophagus).withTimeout(2.0))
        );
    }
}