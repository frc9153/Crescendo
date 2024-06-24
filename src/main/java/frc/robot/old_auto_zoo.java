
//     public final Command m_test = new SequentialCommandGroup(
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new FancyDriveCommand(m_driveSwerve, new Vector2(0.05, 0.05)));
//     public final Command m_redAmpShoot = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.2),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.65),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(1.2),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.3),
//             new ParallelCommandGroup(
//                     new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.5)),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT), // *
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(1.3),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.8),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(0.1),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(2.5) // *
//     );
//     public final Command m_redAmpPiece = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.2),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.7),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(1.2),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.3),
//             new ParallelCommandGroup(new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.5)));
//     public final Command m_redAmpAndDoNothing = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT));

//     public final Command m_blueAmpShoot = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(-90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.2),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.65),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(1.2),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.3),
//             new ParallelCommandGroup(new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.5)),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(1.3),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.8),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(0.1),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(2.5));
//     public final Command m_blueAmpPiece = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(-90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(0, -0.3363), 0, false).withTimeout(0.2),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.7),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(1.2),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.3),
//             new ParallelCommandGroup(new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.5)));
//     public final Command m_blueAmpAndDoNothing = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(-90.0), m_gyro),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0.3363), 0, false).withTimeout(0.7),
//             new AmpThenScore(m_upDown, m_archerfish, m_esophagus).withTimeout(1.0),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT));

//     public final Command m_ShootShoot = new SequentialCommandGroup(
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.3),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.7),
//             new ParallelCommandGroup(new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new SequentialCommandGroup(new WaitCommand(0.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.0))),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new DriveCommand(m_driveSwerve, new Vector2(-0.3363, 0), 0, false).withTimeout(1.2),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(2.5));
//     public final Command m_ShootPiece = new SequentialCommandGroup(
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.3),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.7),
//             new ParallelCommandGroup(new IntakeCommand(m_esophagus).withTimeout(1.5),
//                     new SequentialCommandGroup(new WaitCommand(0.5),
//                     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.0))),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT));
//     public final Command m_ShootAndDoNothing = new SequentialCommandGroup(
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.3));

//     public final Command m_rightShootMobility = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(60.0), m_gyro),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.7),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(0.65),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.0));
//     public final Command m_rightShootCorrect = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(60.0), m_gyro),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.7));
//             //new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.5),
//             //new DriveCommand(m_driveSwerve, new Vector2(0, 0), -0.5, false).withTimeout(0.65),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));
//     public final Command m_leftShootMobility = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(-60.0), m_gyro),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.7),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(0.65),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.0));
//     public final Command m_leftShootCorrect = new SequentialCommandGroup(
//             new InstantCommand(() -> m_gyro.fakeReset(-60.0), m_gyro),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.1),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new WindThenScore(m_archerfish, m_esophagus).withTimeout(0.7));
//             //new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(0.5),
//             //new DriveCommand(m_driveSwerve, new Vector2(0, 0), 0.5, false).withTimeout(0.65),
//             //new InstantCommand(() -> m_driveSwerve.zeroHeading(), m_driveSwerve));

//     public final Command m_mobility = new SequentialCommandGroup(
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             new WaitCommand(0.5),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new WaitCommand(0.5),
//             new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.5)
//     );

//     public final Command m_shootCommandPiece = new SequentialCommandGroup(
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.AMP),
//             new WaitCommand(0.7),
//             // new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
//             // new WaitCommand(0.7),
//             new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
//             new ParallelCommandGroup(new InstantCommand(() -> m_archerfish.startSpin(), m_archerfish),
//                     new WaitCommand(2.0)),
//             new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
//                     new WaitCommand(0.7)),
//             new ParallelCommandGroup(new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
//                     new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish)),
//             new ParallelCommandGroup(new InstantCommand(() -> m_archerfish.slowSpin(), m_archerfish),
//                     new WaitCommand(2.0)),
//             new InstantCommand(() -> m_archerfish.stopSpin(), m_archerfish));

//     public final Command m_FutureShootPiece = new SequentialCommandGroup(
        //     m_shootCommandPiece,
        //     new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.INTAKE),
        //     new InstantCommand(() -> m_esophagus.startFeeding(), m_esophagus),
        //     new DriveCommand(m_driveSwerve, new Vector2(0.3363, 0), 0, false).withTimeout(1.8),
        //     new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus),
        //     new ParallelCommandGroup(new UpDownCommand(m_upDown, Constants.UpDownForever.Setpoint.SHOOT),
        //             new InstantCommand(() -> m_esophagus.startReverse(), m_esophagus),
        //             new WaitCommand(0.2)),
        //     new InstantCommand(() -> m_esophagus.stopFeeding(), m_esophagus)
        // );