// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import javax.annotation.OverridingMethodsMustInvokeSuper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.Drive;

public class Base extends SubsystemBase {
  /** Creates a new Base. */
  public static TalonFX leftMasterMotor = new TalonFX(Constants.leftMasterMotor);
  public static TalonFX leftSlaveMotor = new TalonFX(Constants.leftSlaveMotor);
  public static TalonFX rightMasterMotor = new TalonFX(Constants.rightMasterMotor);
  public static TalonFX rightSlaveMotor = new TalonFX(Constants.rightSlaveMotor);

  Solenoid rightGearChanger = new Solenoid(Constants.rightGearChanger);
  Solenoid leftGearChanger = new Solenoid (Constants.leftGearChanger);


  double leftSpeed;
  double rightSpeed;

  public Base() {
    //init PID
    Constants.initFalconPID(leftMasterMotor, 1);
    Constants.initFalconPID(leftSlaveMotor, 1);
    Constants.initFalconPID(rightMasterMotor, 1);
    Constants.initFalconPID(rightSlaveMotor, 1);

    //set inverse
    leftMasterMotor.setInverted(false);
    leftSlaveMotor.setInverted(false);
    rightMasterMotor.setInverted(true);
    rightSlaveMotor.setInverted(true);

    //set motor follow
    leftSlaveMotor.follow(leftMasterMotor);
    rightSlaveMotor.follow(rightMasterMotor);

    //set motor mode
    leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
    rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    rightSlaveMotor.setNeutralMode(NeutralMode.Brake);
  }

  // public void initDefaultCommand(){
  //   setDefaultCommand(new Drive());
  // }

  public void drive(double x_value, double y_value) {
    SmartDashboard.putNumber("x", x_value);
    SmartDashboard.putNumber("y", y_value);
    setVelocityPID();
    if (Robot.judge.isForward){
      if (y_value>=0){
        leftSpeed = ((-y_value - x_value) * Constants.speedConstant);
        rightSpeed = ((-y_value + x_value) * Constants.speedConstant);
        SmartDashboard.putBoolean("drive test", true);
      } else {
        SmartDashboard.putBoolean("drive test", false);
        leftSpeed = (-y_value + x_value) * Constants.speedConstant;
      rightSpeed = (-y_value - x_value) * Constants.speedConstant;
      }

    }else{
      if (y_value>=0){
        leftSpeed = ((y_value - x_value) * Constants.speedConstant);
        rightSpeed = ((y_value + x_value) * Constants.speedConstant);
        SmartDashboard.putBoolean("drive test", true);
      } else {
        SmartDashboard.putBoolean("drive test", false);
        leftSpeed = (y_value + x_value) * Constants.speedConstant;
      rightSpeed = (y_value - x_value) * Constants.speedConstant;
      }
    } 
    speedSetup();
  }

  public void speedSetup() {
    // setSpeedMode();
    leftMasterMotor.set(TalonFXControlMode.Velocity, leftSpeed);
    rightMasterMotor.set(TalonFXControlMode.Velocity, rightSpeed);
  }

  public void setVelocityPID(){
    Constants.setFalconPID(leftMasterMotor, 0, 0.05, 0, 0);
    Constants.setFalconPID(rightMasterMotor, 0, 0.05, 0, 0);
  }

  public void turn(int angle){
    int requiredDistance=angle;
    leftMasterMotor.setSelectedSensorPosition(0);
    rightMasterMotor.setSelectedSensorPosition(0);
    leftMasterMotor.set(TalonFXControlMode.Position, requiredDistance);
    rightMasterMotor.set(TalonFXControlMode.Position, -requiredDistance);
  }

  public void setSpeedMode(){
    if (Robot.judge.isHighSpeed == true){
      rightGearChanger.set(true);
      leftGearChanger.set(true);
    } else {
      rightGearChanger.set(false);
      rightGearChanger.set(false);
    }
  }

  @Override
  public void periodic() {
    //show velocity
    SmartDashboard.putNumber("left Velocity", leftMasterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("right Velocity", rightMasterMotor.getSelectedSensorVelocity());

    SmartDashboard.putNumber("left Target Velocity", leftSpeed);
    SmartDashboard.putNumber("right Target Velocity", rightSpeed);
    
    //show position
    SmartDashboard.putNumber("left Position", leftMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("right Position", leftMasterMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("xValue", Robot.oi.motionStick.getRawAxis(1));

    //drfault drive
    double xValue = Robot.judge.deadBand(Robot.oi.motionStick.getX());
    double yValue = Robot.judge.deadBand(Robot.oi.motionStick.getY());
    if (Robot.judge.isManualDrive){
      Robot.base.drive(xValue,yValue);
      // leftMasterMotor.set(ControlMode.PercentOutput, xValue);
      // rightMasterMotor.set(ControlMode.PercentOutput, yValue);
    }
  }
}