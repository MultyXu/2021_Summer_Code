// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import javax.annotation.OverridingMethodsMustInvokeSuper;
import javax.security.auth.kerberos.KerberosKey;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  DoubleSolenoid gearChanger = new DoubleSolenoid(0,0,1);
  

  double leftSpeed;
  double rightSpeed;

  int i =0;

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
    leftMasterMotor.setNeutralMode(NeutralMode.Coast);
    leftSlaveMotor.setNeutralMode(NeutralMode.Coast);
    rightMasterMotor.setNeutralMode(NeutralMode.Coast);
    rightSlaveMotor.setNeutralMode(NeutralMode.Coast);
  }

  // public void initDefaultCommand(){
  //   setDefaultCommand(new Drive());
  // }

  public void drive(double x_value, double y_value) {
    SmartDashboard.putNumber("x", x_value);
    SmartDashboard.putNumber("y", y_value);
    setVelocityPID();
    if (Robot.judge.isForward){
      if (y_value<=0){
        leftSpeed = (-y_value + x_value) * Constants.speedConstant;
        rightSpeed = (-y_value - x_value) * Constants.speedConstant;
        SmartDashboard.putBoolean("drive test", true);
      } else {
        SmartDashboard.putBoolean("drive test", false);
        leftSpeed = (-y_value - x_value) * Constants.speedConstant;
        rightSpeed = (-y_value + x_value) * Constants.speedConstant;
      }

    }else{
      if (y_value<=0){
        leftSpeed = ((y_value + x_value) * Constants.speedConstant);
        rightSpeed = ((y_value - x_value) * Constants.speedConstant);
        SmartDashboard.putBoolean("drive test", true);
      } else {
        SmartDashboard.putBoolean("drive test", false);
        leftSpeed = (y_value - x_value) * Constants.speedConstant;
      rightSpeed = (y_value + x_value) * Constants.speedConstant;
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

  public void setPositionPID(){
    Constants.setFalconPID(leftMasterMotor, 0, 0.1, 0, 0);
    Constants.setFalconPID(rightMasterMotor, 0, 0.1, 0, 0);
  }

  public void turn(int angle){
    int requiredDistance=angle;
    setPositionPID();
    leftMasterMotor.set(TalonFXControlMode.Position, requiredDistance);
    rightMasterMotor.set(TalonFXControlMode.Position, -requiredDistance);
  }

  public void forward(int _distance){
    setPositionPID();
    leftMasterMotor.set(TalonFXControlMode.Position, _distance);
    rightMasterMotor.set(TalonFXControlMode.Position, _distance);
  }

  public void resetSensor(){
    leftMasterMotor.setSelectedSensorPosition(0);
    rightMasterMotor.setSelectedSensorPosition(0);
  }

  public void setSpeedMode(){
    if (Robot.judge.isHighSpeed == true){
      gearChanger.set(Value.kForward);
    } else {
      gearChanger.set(Value.kReverse);
    }
  }

  public void stopMotor(){
    leftMasterMotor.set(ControlMode.PercentOutput, 0);
    rightMasterMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean disDrive(int dis, int angle, int speed){
    int tempPos; 
    boolean result; 
    tempPos = dis + angle;
    if (Math.abs(leftMasterMotor.getSelectedSensorPosition())<Math.abs(tempPos)){
      if (tempPos<0){
        leftMasterMotor.set(ControlMode.Velocity, -speed);
      } else {
        leftMasterMotor.set(ControlMode.Velocity, speed);
      }

      result = false;
    } else {
      leftMasterMotor.set(ControlMode.Velocity, 0);
      result = true; 
    }

    tempPos = dis - angle;
    if (Math.abs(rightMasterMotor.getSelectedSensorPosition())<Math.abs(tempPos)){
      if(tempPos<0){
        rightMasterMotor.set(ControlMode.Velocity, -speed);
      } else {
        rightMasterMotor.set(ControlMode.Velocity, speed);
      }

      return false;
    } else {
      rightMasterMotor.set(ControlMode.Velocity, 0);
      return result & true;
    }
  }
  public void test(){
    SmartDashboard.putNumber("i", i);
    if (i<2){
      switch(i){
        case 0:
          if(disDrive(10000, 0, 1000)){
            i++;
            resetSensor();
          }
          break;
        
        case 1:
          if(disDrive(-10000,0,1000)){
            i++;
            resetSensor();
          }
          break;    
      }
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

    // compressor.enabled();
  }
}