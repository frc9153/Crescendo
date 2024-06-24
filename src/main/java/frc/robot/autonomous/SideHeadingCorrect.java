// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TransformableGyro;

public class SideHeadingCorrect extends SequentialCommandGroup {

    public SideHeadingCorrect(TransformableGyro m_gyro, double right_or_left) {
        addCommands(
            new InstantCommand(() -> m_gyro.fakeReset(60.0*right_or_left), m_gyro)
        );
    }
}