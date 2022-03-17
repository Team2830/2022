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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RamseteFactory;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new DriveForwardTest. */
  public ThreeBall() {
     ///Add your commands in the addCommands() call, e.g.
    /// addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(), 
      new ParallelDeadlineGroup(
        new WaitCommand(.25), 
        new ShooterMax(RobotContainer.getInstance().getShooter())),
      new ParallelDeadlineGroup(
        RamseteFactory.createRamseteCommand(
          TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(0, 0, new Rotation2d(0)),
              new Pose2d(Units.inchesToMeters(42), Units.inchesToMeters(0), new Rotation2d())),
              RamseteFactory.getTrajectoryConfig())
        ),
        new IntakeDown(RobotContainer.getInstance().getIntake(),RobotContainer.getInstance().getConveyor())

      ),
      new ParallelDeadlineGroup(
        RamseteFactory.createRamseteCommand(
          TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(Units.inchesToMeters(42), Units.inchesToMeters(0), new Rotation2d()),
              new Pose2d(Units.inchesToMeters(-59), Units.inchesToMeters(18), Rotation2d.fromDegrees(-24))),
              RamseteFactory.getTrajectoryConfig().setReversed(true))
        )),

        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new Shoot(RobotContainer.getInstance().getConveyor(),RobotContainer.getInstance().getIntake())
          ),
      new ParallelDeadlineGroup(
        RamseteFactory.createRamseteCommand(
          TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(Units.inchesToMeters(-59), Units.inchesToMeters(18), Rotation2d.fromDegrees(-24)),
              new Pose2d(Units.inchesToMeters(-16), Units.inchesToMeters(-100), Rotation2d.fromDegrees(-90))),
              RamseteFactory.getTrajectoryConfig())
        ),
        new IntakeDown(RobotContainer.getInstance().getIntake(),RobotContainer.getInstance().getConveyor())
      ),
      new ParallelDeadlineGroup(
        RamseteFactory.createRamseteCommand(
          TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(Units.inchesToMeters(-16), Units.inchesToMeters(-100), Rotation2d.fromDegrees(-90)),
              new Pose2d(Units.inchesToMeters(-59), Units.inchesToMeters(6), Rotation2d.fromDegrees(-24))),
              RamseteFactory.getTrajectoryConfig().setReversed(true))
        ),
        new IntakeDown(RobotContainer.getInstance().getIntake(),RobotContainer.getInstance().getConveyor())
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new Shoot(RobotContainer.getInstance().getConveyor(),RobotContainer.getInstance().getIntake())
        ),

      
      new PrintCommand("EO is Cool"));
  }
}
