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
  private final AnalogInput m_LeftTopPhotoEye = new AnalogInput(Constants.Conveyor.kLeftTopPhotoEye);
  private final AnalogInput m_RightTopPhotoEye = new AnalogInput(Constants.Conveyor.kRightTopPhotoEye);
  private final AnalogInput m_LeftBottomPhotoEye = new AnalogInput(Constants.Conveyor.kLeftBottomPhotoEye);
  private final AnalogInput m_RightBottomPhotoEye = new AnalogInput(Constants.Conveyor.kRightBottemPhotoEye);



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

  public boolean getLeftTopPhotoEye() {
    //System.out.println("Left Top Photoeye: " + m_LeftTopPhotoEye.getVoltage());
    SmartDashboard.putNumber("LeftPhotoeye", m_LeftTopPhotoEye.getVoltage());
    return m_LeftTopPhotoEye.getVoltage()<.2; 
  }

  public boolean getLeftBottomPhotoEye() {
    //System.out.println("Left Bottem Photoeye: " + m_LeftBottomPhotoEye.getVoltage());
    return m_LeftBottomPhotoEye.getVoltage()< .55;
  }
  public boolean getRightTopPhotoEye() {
    //System.out.println("Right Top Photoeye: " + m_RightTopPhotoEye.getVoltage());
    return m_RightTopPhotoEye.getVoltage()<.55; 
  }
  
  public boolean getRightBottomPhotoEye() {
    //System.out.println("Right Bottem Photoeye: " + m_RightBottomPhotoEye.getVoltage());
    SmartDashboard.putNumber("RightPhotoeye", m_RightBottomPhotoEye.getVoltage());
    return m_RightBottomPhotoEye.getVoltage()> .5;

  }

  public boolean getTopPhotoEye() {
    return getLeftTopPhotoEye();
  }
  public boolean getBottomPhotoEye() {
    return getRightBottomPhotoEye();
  }
}
