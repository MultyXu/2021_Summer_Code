package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.networktables.*;
import frc.robot.Robot;


public class Networktable extends SubsystemBase {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // modify needed check the networktale entry nuner
  
    NetworkTable sender = inst.getTable("vision");
    NetworkTableEntry isNeeded = sender.getEntry("isNeeded");//send is needed
    NetworkTableEntry shooterAngle = sender.getEntry("shooterAngle");//send shooter angle
    NetworkTableEntry adjustAngle = sender.getEntry("toss_adjustment");//读取视觉角度 （炮台需要调整的角度）
    NetworkTableEntry horizontalDistance = sender.getEntry("a_adjustment");//读取车需要横向移动的距离
    NetworkTableEntry targetDistance = sender.getEntry("distance");//读取车距离洞口的距离

    public int shooterTargetAngle;
    public int horizontalTargetDistance;

    double x =0;//isNeeded

  public Networktable() {
    inst.startClientTeam(7280);
    inst.startDSClient();
  }

  public void getTableData() {

    shooterTargetAngle = (int)adjustAngle.getDouble(0.0);
    horizontalTargetDistance = (int)horizontalDistance.getDouble(0.0);

    if(Robot.judge.tableOn){
      x = 1.0;
    } else {
      x=0.0;
    }
    isNeeded.setDouble(x);
    shooterAngle.setDouble(Robot.shooter.shooterCurrentAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTableData();

    //show data
  }
}
