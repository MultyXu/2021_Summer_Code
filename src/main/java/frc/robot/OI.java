// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;


/** Add your docs here. */
public class OI {
    public static Joystick motionStick = new Joystick(Constants.motionJoystick);
    public static Joystick rotateStick = new Joystick(Constants.rotateJoystick);
    public JoystickButton shootBall = new JoystickButton(motionStick, 1);
    public JoystickButton intake = new JoystickButton(motionStick, 2);
    public JoystickButton changeDriveMode = new JoystickButton(motionStick, 3);
    public JoystickButton changeDriveDirection = new JoystickButton(motionStick,4);
    public JoystickButton test = new JoystickButton(motionStick, 5);
    public JoystickButton test2 = new JoystickButton(motionStick, 6);
    public JoystickButton autoTest = new JoystickButton(motionStick, 9);
    public JoystickButton climbTest = new JoystickButton(motionStick, 10);
    
    public OI(){
        // shootBall.whenHeld(new ShootBall());
        changeDriveMode.whenPressed(new ChangeDriveMode());
        changeDriveDirection.whenPressed(new ChangeDriveDirection());
        test.whenHeld(new Test());
        test2.whenHeld(new Test2());
        autoTest.whenPressed(new Forward(0));
        climbTest.whenPressed(new ClimbTest());
    }
}