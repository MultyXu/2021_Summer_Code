// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new SpinningCollect. */
  public static TalonFX intakeMotor = new TalonFX(Constants.intakeMotor);

  Solenoid intakeMasterSolenoid = new Solenoid(Constants.intakeMasterSolenoid);
  Solenoid intakeSlaveSolenoid = new Solenoid(Constants.intakeSlaveSolenoid);
  public Intake() {
    Constants.initFalconPID(intakeMotor, 1);
  }

  public void intake(){
    intakeMasterSolenoid.set(true);
    intakeSlaveSolenoid.set(true);
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}