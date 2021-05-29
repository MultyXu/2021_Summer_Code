// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;

/** Add your docs here. */

public class Judge extends SubsystemBase{
    public boolean isShooting = false;
    public boolean isForward = true;
    public boolean isTargeting = false;
    public boolean isHighSpeed = true;
    public boolean isRotating = false;
    public boolean isPreparing = true;
    public boolean isManualDrive = true;
    public boolean isManualRotate = false;
    public boolean tableOn = false;
    public boolean isIntaking = false;
    public boolean isReady = false;

    // public void isForwardDetecting(){
    //     if (Constants.yValue >= 0){
    //         isForward = true;
    //     }
    //     else if (Constants.yValue < 0){
    //         isForward = false;
    //     }
    // }

    public void changeDriveDirection(){
        isForward=!isForward;
    }

    public void changeShootState(){
        isShooting = !isShooting;
    }

    public void changeSpeedMode(){
        isHighSpeed=!isHighSpeed;
    }

    public static double deadBand(double value){
        //init joystick output dataa
        if (value >= 0.08){
            return value;
        }

        if (value <= -0.08){
            return value;
        }

        return 0;
    }

    public void detecting(){
        //ball keeper rotate
        if(Robot.fort.rotateMotor.getSelectedSensorVelocity()>10){
            isRotating = true;
        } else {
            isRotating = false;
        }

        if (Robot.oi.motionStick.getRawAxis(3)>0.5){
            isPreparing=true;
        } else {
            isPreparing = false;
        }

        if (Robot.oi.rotateStick.getRawButton(7)){
            isManualRotate = true;
        } else {
            isManualRotate = false;
        }

        if (Robot.oi.rotateStick.getRawButton(1)){
            isShooting=true;
        } else {
            isShooting = false;
        }

        tableOn = Robot.oi.motionStick.getRawButton(11);

        if (Robot.shooter.shooterMasterMotor.getSelectedSensorVelocity()>15000){
            isReady = true;
        } else {
            isReady = false;
        }

    }

    public void test(){
        // if (OI.motionStick.getRawButton(1) == true){
        //     SmartDashboard.putBoolean("test", true);
        // } else {
        //     SmartDashboard.putBoolean("test", false);
        // }
    }

    @Override
    public void periodic(){
        detecting();
        SmartDashboard.putBoolean("isShooting", isShooting);
        SmartDashboard.putBoolean("isForward", isForward);
        SmartDashboard.putBoolean("isHighSpeed", isHighSpeed);
        SmartDashboard.putBoolean("isTargeting", isTargeting);
        SmartDashboard.putBoolean("isRotating", isRotating);
        SmartDashboard.putBoolean("isPreparing", isPreparing);
        SmartDashboard.putBoolean("isManualDrive", isManualDrive);
        SmartDashboard.putBoolean("isManualRotate", isManualRotate);
        SmartDashboard.putBoolean("isIntaking", isIntaking);
        SmartDashboard.putBoolean("tableOn", tableOn);
    }
    
}