// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Fort extends SubsystemBase {
  /** Creates a new Battery. */
  
  public static TalonFX fortController = new TalonFX(Constants.fortMotor);
  public static TalonFX rotateMotor = new TalonFX(Constants.rotateMotor);

  public double fortRotateSpeed;

  

  public Fort() {
    Constants.initFalconPID(fortController, 1);
    Constants.initFalconPID(rotateMotor, 1);

    fortController.setNeutralMode(NeutralMode.Brake);
    rotateMotor.setNeutralMode(NeutralMode.Coast);

  }

  public void rotateFort(double _percent){
    fortController.set(TalonFXControlMode.PercentOutput, _percent);
  }

  public void configVelocityPID(){
    Constants.setFalconPID(fortController, 0, 0, 0, 0);
  }


  public void setPositionPID(){
    Constants.setFalconPID(fortController, 0, 0.01, 0, 0);

  }

  public void rotateToAngle(double _angle){
    setPositionPID();
    double targetPosition = _angle;
    fortController.set(TalonFXControlMode.Position, targetPosition);
  }

  public void rotateBallKeeper(){
    rotateMotor.set(TalonFXControlMode.Velocity, Constants.ballKeeperRotateSpeed);
  }

  public void stopBallKeeper(){
    rotateMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void fortToAngle(int _position){
    setPositionPID();
    fortController.set(ControlMode.Position, _position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.judge.isManualRotate){
      if (Robot.oi.motionStick.getPOV()==270){
        rotateFort(0.1);
      } else if (Robot.oi.motionStick.getPOV()==90){
        rotateFort(-0.1);
      } else {
        rotateFort(0);
      }
    } else if (Robot.judge.tableOn){
      if (Robot.networktable.fortTargetAngle >1){
        rotateFort(-0.1);
      } else if (Robot.networktable.fortTargetAngle <-1){
        rotateFort(0.1);
      } else {
        rotateFort(0);
      }
    } else {
      rotateFort(0);
    }


    SmartDashboard.putNumber("POV", Robot.oi.motionStick.getPOV());
    SmartDashboard.putNumber("fortPos", fortController.getSelectedSensorPosition());

    if (Robot.judge.isShooting){
      rotateMotor.set(ControlMode.PercentOutput, 0.2);
    } else if (Robot.judge.isIntaking){
      rotateMotor.set(ControlMode.PercentOutput, 0.2);
    } else {
      rotateMotor.set(ControlMode.PercentOutput, 0);
    }


  }


}
