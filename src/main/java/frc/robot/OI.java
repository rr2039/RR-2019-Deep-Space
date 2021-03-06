/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.commands.*;
import frc.robot.Deadzone;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  ClimbingPistons climbPistons = new ClimbingPistons();
  Joystick driveJoy = new Joystick(0);
  Joystick operatorJoy = new Joystick(1);

  Button wristUp = new JoystickButton(operatorJoy, 6);
  Button wristDown = new JoystickButton(operatorJoy, 7);

  Boolean climb = driveJoy.getRawButtonPressed(100); //100 is a placeholder until an official button is determined
  Boolean lower = driveJoy.getRawButtonPressed(200); //Same as above, 200 is a placeholder.


  double driveLeftStick_X = driveJoy.getRawAxis(0);
  double driveLeftStick_Y = driveJoy.getRawAxis(1);
  double driveRightStick_X = driveJoy.getRawAxis(4);
  double driveRightStick_Y = driveJoy.getRawAxis(5);

  MecanumDrive mecDrive = new MecanumDrive(RobotMap.talonFL, RobotMap.talonBL, RobotMap.talonFR, RobotMap.talonBR);

  public OI() {
    wristUp.whenPressed(new wristUp());
    wristDown.whenPressed(new wristDown());
    Deadzone.whenOutside(new moveLift(operatorJoy.getRawAxis(1)), operatorJoy.getRawAxis(1), 0.15);
    mecDrive.driveCartesian(driveLeftStick_X, -driveLeftStick_Y, driveRightStick_X);

    if (climb) {
      climbPistons.extend();
    }
    else if (lower) {
      climbPistons.retract();   
    }
    else {
      // Check the climbPistons class for a recursion joke.
      // climbPistons.sitThereAndLookPretty();
    }
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.()
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
