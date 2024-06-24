// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TransformableGyro;

public class AmpHeadingCorrect extends SequentialCommandGroup {

    public AmpHeadingCorrect(TransformableGyro m_gyro, double red_or_blue) {
        addCommands(
            new InstantCommand(() -> m_gyro.fakeReset(90.0*red_or_blue), m_gyro)
        );
    }
}