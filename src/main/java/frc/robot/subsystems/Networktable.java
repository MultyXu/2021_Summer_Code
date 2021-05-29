package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.networktables.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Networktable extends SubsystemBase {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTableInstance inst1 = NetworkTableInstance.getDefault();
    // modify needed check the networktale entry nuner
  
    NetworkTable sender = inst.getTable("vision");
    NetworkTable limeLight = inst1.getTable("limelight");
    NetworkTableEntry tv = limeLight.getEntry("tv"); 
    NetworkTableEntry tx = limeLight.getEntry("tx"); 
    NetworkTableEntry ty = limeLight.getEntry("ty"); 
    NetworkTableEntry ta = limeLight.getEntry("ta"); 


    public int shooterTargetAngle;
    public int fortTargetAngle;

    public double v = 0.0;
    public double x = 0.0;
    public double y = 0.0;
    public double a = 0.0;
    public double distance = 0.0;

    public double angle = 0.0;

  public Networktable() {
    inst.startClientTeam(7280);
    inst.startDSClient();
  }

  public void getTableData() {

    v = tv.getDouble(0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);

    angle = Robot.shooter.shooterCurrentAngle;
    if (v ==0){
      distance = (2.32-0.70859*(1+0.31439)*Math.sin(angle))/Math.tan(-y*2*Math.PI/360+(90-angle)*2*Math.PI/360);
    }

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTableData();

    // SmartDashboard.putNumber("shooter", shooterTargetAngle);
    // SmartDashboard.putNumber("fort", fortTargetAngle);
    // SmartDashboard.putNumber("base", distance);
    // SmartDashboard.putNumber("tableTable", x);

    //show data
    SmartDashboard.putNumber("tv", v);
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
    SmartDashboard.putNumber("ta", a);
    SmartDashboard.putNumber("distance", distance);
  }
}
