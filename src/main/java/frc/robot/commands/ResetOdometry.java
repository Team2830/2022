// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RamseteFactory;

/** Add your docs here. */
public class ResetOdometry extends edu.wpi.first.wpilibj2.command.InstantCommand {
  /** Add your docs here. */
  public ResetOdometry() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    RamseteFactory.resetOdometryToZeros();
  }
}
