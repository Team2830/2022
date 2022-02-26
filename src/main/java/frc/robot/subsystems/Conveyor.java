// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final WPI_TalonFX m_ConveryorRoller = new WPI_TalonFX(Constants.Conveyor.kConveyorId);
  private final DigitalInput m_TopPhotoEye = new DigitalInput(Constants.Conveyor.kTopPhotoEye);
  private final DigitalInput m_BottomPhotoEye = new DigitalInput(Constants.Conveyor.kBottomPhotoEye);
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

  public boolean getTopPhotoEye() {
    return m_TopPhotoEye.get();
  }
  public boolean getBottomPhotoEye() {
    return m_BottomPhotoEye.get();
  }
  
}
