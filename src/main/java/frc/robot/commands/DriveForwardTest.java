// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RamseteFactory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardTest extends SequentialCommandGroup {
  /** Creates a new DriveForwardTest. */
  public DriveForwardTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(), 
      RamseteFactory.createRamseteCommand(
        TrajectoryGenerator.generateTrajectory(
          List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(Units.feetToMeters(10), 0, new Rotation2d(0))),
            RamseteFactory.getTrajectoryConfig())
      ),
      new PrintCommand("EO is Cool")
    );
  }
}
