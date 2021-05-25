// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Judge;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  public Drive() {
    addRequirements(Robot.base);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xValue = Robot.judge.deadBand(Robot.oi.motionStick.getX());
    double yValue = Robot.judge.deadBand(Robot.oi.motionStick.getY());

    Robot.base.drive(xValue,yValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


}