// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX m_roller = new WPI_TalonFX(Constants.IntakeConstants.KIntakeRollerId);
  private final DoubleSolenoid m_Solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.kPneumaticUpId, Constants.IntakeConstants.kPneumaticDownId);

  /** Creates a new Intake. */
  public Intake() {
    m_roller.configFactoryDefault();
    m_roller.setInverted(TalonFXInvertType.Clockwise);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeIn() {
    m_roller.set(TalonFXControlMode.PercentOutput, 1);
  }

  public void intakeOut(){
    m_roller.set(TalonFXControlMode.PercentOutput, -1);
  }

  public void intakeStop(){
    m_roller.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void intakeUp(){
    m_Solenoid.set(Value.kForward);
  }

  public void intakedown(){
    m_Solenoid.set(Value.kReverse);
  }



}
