// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX m_SmallRoller = new WPI_TalonFX(Constants.Shooter.kSmallShooterId);
  private final WPI_TalonFX m_BigRoller = new WPI_TalonFX(Constants.Shooter.kBigShotterId);
  private final WPI_TalonFX m_FollowerRoller = new WPI_TalonFX(Constants.Shooter.kBigShotterFollowerId);


  /** Creates a new Shooter. */
  public Shooter() {
    m_SmallRoller.configFactoryDefault();
    m_BigRoller.configFactoryDefault();
    m_FollowerRoller.configFactoryDefault();
    m_SmallRoller.setInverted(TalonFXInvertType.Clockwise);
    m_BigRoller.setInverted(TalonFXInvertType.CounterClockwise);
    m_FollowerRoller.follow(m_BigRoller);
    m_FollowerRoller.setInverted(TalonFXInvertType.OpposeMaster);
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void shooterMax(){
    m_SmallRoller.set(TalonFXControlMode.PercentOutput, 1);
    m_BigRoller.set(TalonFXControlMode.PercentOutput, 1);
  }

  public void shooterSlow(){
    m_SmallRoller.set(TalonFXControlMode.PercentOutput, .5);
    m_BigRoller.set(TalonFXControlMode.PercentOutput, .5);
  }
  public void shooterStop(){
    m_SmallRoller.set(TalonFXControlMode.PercentOutput, 0);
    m_BigRoller.set(TalonFXControlMode.PercentOutput, 0);
  }



  }