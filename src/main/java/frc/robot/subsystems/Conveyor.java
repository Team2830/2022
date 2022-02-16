// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final WPI_TalonFX m_ConveryorRoller = new WPI_TalonFX(Constants.Conveyor.kConveyorId);
  /** Creates a new Conveyor. */
  public Conveyor() {
    m_ConveryorRoller.configFactoryDefault();
    m_ConveryorRoller.setInverted(TalonFXInvertType.CounterClockwise);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void ConveyorUp(){
    m_ConveryorRoller.set(TalonFXControlMode.PercentOutput, 1);
  }
  public void Conveyordown(){
    m_ConveryorRoller.set(TalonFXControlMode.PercentOutput, -1);
  }
  public void ConveyorStop(){
    m_ConveryorRoller.set(TalonFXControlMode.PercentOutput, 0);

  }
}
