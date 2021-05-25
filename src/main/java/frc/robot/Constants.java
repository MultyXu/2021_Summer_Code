// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Talon;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //basic constant 
    public static final int kTimeoutMs = 30;

    public static final int kPIDLoopIdx = 0;

    public static final int kSlotIdx = 0;

    public static boolean kSensorPhase = true;

    public static boolean kMotorInverted = true;

    //motionstick
    public final static int motionJoystick = 0;
    public final static int rotateJoystick = 1;

    //shooter speed
    public final static int shootingMaster = 1;
    public final static int shootingSlaver = 2;
    public final static int shootVelocity = 6500;

    //PID
    public static void initFalconPID(TalonFX _talon, double _peakOutput){
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

        _talon.configFactoryDefault();
        _talon.setSensorPhase(true);

        _talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1, kTimeoutMs);
		_talon.configPeakOutputReverse(-1, kTimeoutMs);

        _talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    
        _talon.configAllowableClosedloopError(1, kPIDLoopIdx, kTimeoutMs);
    
        _talon.configClosedLoopPeakOutput(kSlotIdx, _peakOutput, kTimeoutMs);

    }

    public static void setFalconPID(TalonFX _talon, double kF, double kP, double kI, double kD){
        _talon.config_kF(kSlotIdx, kF);
        _talon.config_kP(kSlotIdx, kP);
        _talon.config_kI(kSlotIdx, kI);
        _talon.config_kD(kSlotIdx, kD);
    }
    
    //motorport

    //base
    public final static int leftMasterMotor = 1;
    public final static int leftSlaveMotor = 2;
    public final static int rightMasterMotor = 3;
    public final static int rightSlaveMotor = 4;

    //fort 
    public final static int rotateMotor = 10;
    public final static int fortMotor = 5;
    
    //shooter
    public final static int shooterMasterMotor = 6;
    public final static int shooterSlaveMotor = 7;
    public final static int ballDeliverMotor = 9;
    public final static int shooterAngleMotor = 8;

    //intake
    public final static int intakeMotor = 11;

    //climb
    public final static int climbMasterMotor = 12;
    public final static int climbSlaveMotor = 13;


    //solenoid
    //base
    public final static int rightGearChanger = 1;
    public final static int leftGearChanger = 2;

    //shooter
    public final static int ballDeliverSolenoid = 3;

    //intake
    public final static int intakeMasterSolenoid = 4; 
    public final static int intakeSlaveSolenoid = 5;


 
    //motorspeed/position
    //base
    public final static int speedConstant = 15000;

    public static double spinning_value = 200;
    public static double litInput;
    public static double litTarget;
    public static double givingBall_speed;

    //fort
    public final static int ballKeeperRotateSpeed = 1000;
    public final static int fortRotateSpeedConstant = 50;

    //shooter
    public final static int flywheelVelocity = 10000;

    //Constants
    public final static int climbSpeed = 1000;
}