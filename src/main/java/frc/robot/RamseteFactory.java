// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class RamseteFactory {
    private static DriveSubsystem robotDrive = null;

    public static void setRobotDrive(DriveSubsystem drive) {
        if(robotDrive == null)
            robotDrive = drive;
    }

    public static void resetOdometryToZeros() {
        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(new Pose2d());
    }

    public static RamseteCommand createRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
            trajectory,
            robotDrive::getPose, 
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
            new SimpleMotorFeedforward(
                  DriveConstants.ksVolts,
                  DriveConstants.kvVoltSecondsPerMeter,
                  DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics, 
            robotDrive::getWheelSpeeds, 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            new PIDController(DriveConstants.kPDriveVel, 0, 0), 
            robotDrive::tankDriveVolts, 
            robotDrive);
    }

    public static TrajectoryConfig getTrajectoryConfig() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);
    
        // Create config for trajectory
        return new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
      }
}
