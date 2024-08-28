// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TransformableGyro;

public class CenterHeadingCorrect extends SequentialCommandGroup {

    public CenterHeadingCorrect(TransformableGyro m_gyro) {
        addCommands(
            new InstantCommand(() -> m_gyro.reset(), m_gyro)
        );
    }
}