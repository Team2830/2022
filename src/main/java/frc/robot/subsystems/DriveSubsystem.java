// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase implements Loggable  {
  // The motors on the left side of the drive.
  private final WPI_TalonFX m_LeftMotor = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_LeftFollowerMotor = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
  private final WPI_TalonFX m_RightMotor = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_RightFollowerMotor = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter);

  private final PIDController m_leftPIDController = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);

  private final DifferentialDriveKinematics m_Kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);

  // The robot's drive
  @Log.DifferentialDrive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_LeftMotor, m_RightMotor);

  // The gyro sensor
  @Log.Gyro
  private final Gyro m_gyro = new AHRS();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. 
   * @param InvertType */
  public DriveSubsystem() {
    m_RightMotor.configFactoryDefault();
    m_RightFollowerMotor.configFactoryDefault();
    m_LeftMotor.configFactoryDefault();
    m_LeftFollowerMotor.configFactoryDefault();
    

    m_LeftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,40,0));
    m_RightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,40,0));
    
    m_LeftFollowerMotor.follow(m_LeftMotor);
    m_RightFollowerMotor.follow(m_RightMotor);

    m_RightMotor.setInverted(TalonFXInvertType.Clockwise);
    m_RightFollowerMotor.setInverted(TalonFXInvertType.FollowMaster);
    m_LeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    m_LeftFollowerMotor.setInverted(TalonFXInvertType.FollowMaster);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), 
        m_LeftMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse, 
        m_RightMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse);
        System.out.println("Left: " + m_LeftMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse);
        System.out.println("Right: " + m_RightMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(getWheelSpeeds().leftMetersPerSecond, speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(getWheelSpeeds().rightMetersPerSecond, speeds.rightMetersPerSecond);
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public void closedLoopDrive(double xSpeed, double rot) {
    var wheelSpeeds = m_Kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));
    setSpeeds(wheelSpeeds);
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_LeftMotor.getSelectedSensorVelocity() * 10 * Constants.DriveConstants.kEncoderDistancePerPulse,
     m_RightMotor.getSelectedSensorVelocity() * 10 * Constants.DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    System.out.println("Gyro Reset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_LeftMotor.setVoltage(leftVolts);
    m_RightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_LeftMotor.setSelectedSensorPosition(0);
    m_RightMotor.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_LeftMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse +
     m_RightMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
