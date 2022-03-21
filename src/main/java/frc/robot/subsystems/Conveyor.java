// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final WPI_TalonFX m_ConveryorRoller = new WPI_TalonFX(Constants.Conveyor.kConveyorId);
  private final AnalogInput m_TopPhotoEye = new AnalogInput(Constants.Conveyor.kTopPhotoEye);
  private final AnalogInput m_BottomPhotoEye = new AnalogInput(Constants.Conveyor.kBottomPhotoEye);


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
    //System.out.println("Left Top Photoeye: " + m_LeftTopPhotoEye.getVoltage());
    SmartDashboard.putNumber("TopPhotoeye", m_TopPhotoEye.getVoltage());
    return m_TopPhotoEye.getVoltage()>2; 
  }
  public boolean getBottomPhotoEye() {
    //System.out.println("Right Bottem Photoeye: " + m_RightBottomPhotoEye.getVoltage());
    SmartDashboard.putNumber("BottomPhotoeye", m_BottomPhotoEye.getVoltage());
    return m_BottomPhotoEye.getVoltage()> 2;
  }
}
