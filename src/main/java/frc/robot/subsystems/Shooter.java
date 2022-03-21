// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX m_SmallRoller = new WPI_TalonFX(Constants.Shooter.kSmallShooterId);
  private final WPI_TalonFX m_BigRoller = new WPI_TalonFX(Constants.Shooter.kBigShotterId);
  private final WPI_TalonFX m_FollowerRoller = new WPI_TalonFX(Constants.Shooter.kBigShotterFollowerId);


  /** Creates a new Shooter. */
  public Shooter() {
    m_SmallRoller.configFactoryDefault();
    m_SmallRoller.config_kP(0,Constants.Shooter.kPSmallShooter);
    m_SmallRoller.config_kF(0,Constants.Shooter.kFSmallShooter);
    m_SmallRoller.config_kD(0,Constants.Shooter.kDSmallShooter);
    m_BigRoller.configFactoryDefault();
    m_BigRoller.config_kP(0,Constants.Shooter.kPBigShotter);
    m_BigRoller.config_kF(0,Constants.Shooter.kFBigShotter);
    m_BigRoller.config_kD(0,Constants.Shooter.kDBigShotter);
    m_FollowerRoller.configFactoryDefault();
    m_SmallRoller.setInverted(TalonFXInvertType.CounterClockwise);
    m_BigRoller.setInverted(TalonFXInvertType.CounterClockwise);
    m_FollowerRoller.follow(m_BigRoller);
    m_FollowerRoller.setInverted(TalonFXInvertType.OpposeMaster);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Shooter Speed", m_SmallRoller.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Button Shooter Speed", m_BigRoller.getSelectedSensorVelocity());
  }

  public void shooterMax(){
    m_SmallRoller.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Small Roller High Goal", 8000));
    m_BigRoller.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Big Roller High Goal", 8500));
  }

  public void shooterSlow(){
    m_SmallRoller.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Big Roller Low Goal", 5000));
    m_BigRoller.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Small Roller Low Goal", 5000));
  }
  public void shooterStop(){
    m_SmallRoller.set(TalonFXControlMode.Velocity, 0);
    m_BigRoller.set(TalonFXControlMode.Velocity, 0);
  }
  }