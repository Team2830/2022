// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final WPI_TalonFX m_LeftClimberMotor = new WPI_TalonFX(Constants.ClimberConstants.kLeftClimberMotorId);
  private final WPI_TalonFX m_RightClimberMotor = new WPI_TalonFX(Constants.ClimberConstants.kRightClimberMotorId);
  /** Creates a new Climber. */
  public Climber() {
    m_LeftClimberMotor.configFactoryDefault();
    m_LeftClimberMotor.setInverted(TalonFXInvertType.Clockwise);
    m_RightClimberMotor.configFactoryDefault();
    m_RightClimberMotor.follow(m_LeftClimberMotor);
    m_RightClimberMotor.setInverted(TalonFXInvertType.OpposeMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void normalMovement() {
    m_LeftClimberMotor.set(TalonFXControlMode.PercentOutput, 1);
    m_RightClimberMotor.set(TalonFXControlMode.PercentOutput, 1);
  }

  public void reverseMovement() {
    m_LeftClimberMotor.set(TalonFXControlMode.PercentOutput, -1);
    m_RightClimberMotor.set(TalonFXControlMode.PercentOutput, -1);
  }

  public void stopMovement(){
    m_LeftClimberMotor.set(TalonFXControlMode.PercentOutput, 0);
    m_RightClimberMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return (m_LeftClimberMotor.getSelectedSensorPosition() + m_RightClimberMotor.getSelectedSensorPosition()) / 2;
  }
  
}
