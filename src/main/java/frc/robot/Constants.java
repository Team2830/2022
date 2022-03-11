// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double kTrackwidthMeters = 0.69459;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse =
        ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / 10.71;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.71886;
    public static final double kvVoltSecondsPerMeter = 2.2607;
    public static final double kaVoltSecondsSquaredPerMeter = 0.3051;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 2.4538;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ClimberConstants {
    public static final int kLeftClimberMotorId = 5;
    public static final int kRightClimberMotorId = 6;
    public static final int kTopClimberMotorPosition = 180000;
    public static final int kBottomClimberMotorPosition = 360000;
  }

  public static final class IntakeConstants{
    public static final int KIntakeRollerId = 8;
    public static final int kPneumaticUpId = 0;
    public static final int kPneumaticDownId = 1;

  }

  public static final class Shooter{
    public static final int kSmallShooterId = 13;
    public static final int kBigShotterId = 14;
    public static final int kBigShotterFollowerId = 15;
    public static final double kPSmallShooter = 0.02;
    public static final double kFSmallShooter = 0.0448;
    public static final double kDSmallShooter = 0.0003;
    public static final double kPBigShotter = 0.09;
    public static final double kFBigShotter = 0.0448;
    public static final double kDBigShotter = 0.0009;
  }

  public static final class Conveyor{
    public static final int kConveyorId = 16;
    public static final int kTopPhotoEye = 17;
    public static final int kBottomPhotoEye = 18;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
