// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  /** Creates a new Lit. */

  public static TalonFX shooterMasterMotor = new TalonFX(Constants.shooterMasterMotor);
  public static TalonFX shooterSlaveMotor = new TalonFX(Constants.shooterSlaveMotor);
  public static TalonFX ballDeliverMotor = new TalonFX(Constants.ballDeliverMotor);
  public static TalonFX shooterAngleMotor = new TalonFX(Constants.shooterAngleMotor);

  Solenoid ballDeliverSolenoid = new Solenoid(Constants.ballDeliverSolenoid);

  public int targetPosition;
  public Shooter() {
    Constants.initFalconPID(shooterMasterMotor, 1);
    Constants.initFalconPID(shooterSlaveMotor, 1);
    Constants.initFalconPID(ballDeliverMotor, 1);
    Constants.initFalconPID(shooterAngleMotor, 1);

    shooterMasterMotor.setInverted(true);
    shooterSlaveMotor.setInverted(false);
    shooterAngleMotor.setInverted(false);

    shooterSlaveMotor.follow(shooterMasterMotor);

    shooterMasterMotor.setNeutralMode(NeutralMode.Coast);
    shooterSlaveMotor.setNeutralMode(NeutralMode.Coast);
    shooterAngleMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void startShoot(){
    ballDeliverSolenoid.set(true);
    ballDeliverMotor.set(ControlMode.PercentOutput, 0.1);
    Robot.judge.isShooting = true;
  }

  public void stopShoot(){
    ballDeliverSolenoid.set(false);
    ballDeliverMotor.set(ControlMode.PercentOutput, 0);
    Robot.judge.isShooting = false;
  }

  public void configVelocityPID(){
    Constants.setFalconPID(shooterMasterMotor, 0, 0.1, 0, 0);
    Constants.setFalconPID(shooterSlaveMotor, 0, 0.1, 0, 0);
  }

  public void increaseAngleTest(int _position){
    Constants.setFalconPID(shooterAngleMotor, 0, 0.018, 0, 0);
    shooterAngleMotor.set(ControlMode.Position, _position);
}



public void decreaseAngleTest(int _position){
  Constants.setFalconPID(shooterAngleMotor, 0, 0.018, 0, 0);
  shooterAngleMotor.set(ControlMode.Position, _position);
}

public void angleMotorStop(){
  shooterAngleMotor.set(ControlMode.PercentOutput, 0);
}
   

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("flywheelVelocity", shooterMasterMotor.getSelectedSensorVelocity());
    if (Robot.judge.isPreparing){
      configVelocityPID();
      shooterMasterMotor.set(ControlMode.Velocity, 10000);
      shooterSlaveMotor.set(ControlMode.Velocity, 10000);
    } else {
      shooterMasterMotor.set(ControlMode.PercentOutput, 0);
      shooterSlaveMotor.set(ControlMode.PercentOutput, 0);
    }
    SmartDashboard.putNumber("flywheelTargetVelocity", 5000);
    SmartDashboard.putNumber("shooterAnglePosition", shooterAngleMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("shooterAngleVelocity", shooterAngleMotor.getSelectedSensorVelocity());
    
  }
}
