package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.networktables.*;

public class Networktable extends SubsystemBase {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable put = inst.getTable("tape");
    NetworkTable ball = inst.getTable("ball");
    NetworkTableEntry ballPositioneEntry = ball.getEntry("Y");
    NetworkTableEntry upTapeEntry = put.getEntry("Y"); 
    NetworkTableEntry downTapeEntry = put.getEntry("X");
    NetworkTableEntry distanceEntry = put.getEntry("distance");
    NetworkTableEntry angleEntry = put.getEntry("angle");
    NetworkTableEntry midEntry = put.getEntry("midpoint"); 
  
    // modify needed check the networktale entry nuner
  
    NetworkTable sender = inst.getTable("isNeeded");
    NetworkTableEntry isNeeded = sender.getEntry("X");

  public Networktable() {
    inst.startClientTeam(7280);
    inst.startDSClient();
  }

  public void getTableData() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTableData();

    //show data
  }
}
