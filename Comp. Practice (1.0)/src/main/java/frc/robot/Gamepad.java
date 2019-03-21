package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Gamepad extends Joystick {

  private static final int AxisLeftStickX = 0;
  private static final int AxisLeftStickY = 1;
  private static final int AxisLeftTrigger = 2;
  private static final int AxisRightTrigger = 3;
  private static final int AxisRightStickX = 4;
  private static final int AxisRightStickY = 5;
  
  private static final int ButtonA = 1;
  private static final int ButtonB = 2;
  private static final int ButtonX = 3;
  private static final int ButtonY = 4;
  private static final int ButtonLeftBumper = 5;
  private static final int ButtonRightBumper = 6;
  private static final int ButtonBack = 7;
  private static final int ButtonStart = 8;
  private static final int ButtonLeftStick = 9;
  private static final int ButtonRightStick = 10;



  /**

   * Constructor that creates a Joystick object.

   */

  public Gamepad(int gamepadPort) {

    super(gamepadPort);

  }

  public double getAxisLeftStickX(){
    return getRawAxis(AxisLeftStickX);
  }

  public double getAxisLeftStickY(){
    return getRawAxis(AxisLeftStickY);
  }

  public double getAxisLeftTrigger(){
    return getRawAxis(AxisLeftTrigger);
  }

  public double getAxisRightTrigger(){
    return getRawAxis(AxisRightTrigger);
  }

  public double getAxisRightStickX(){
    return getRawAxis(AxisRightStickX);
  }

  public double getAxisRightStickY(){
    return getRawAxis(AxisRightStickY);
  }



  public boolean getButtonA(){
    return getRawButton(ButtonA);
  }
  public boolean getButtonAPressed(){
    return getRawButtonPressed(ButtonA);
  }

  public boolean getButtonB(){
    return getRawButton(ButtonB);
  }
  public boolean getButtonBPressed(){
    return getRawButtonPressed(ButtonB);
  }

  public boolean getButtonX(){
    return getRawButton(ButtonX);
  }
  public boolean getButtonXPressed(){
    return getRawButtonPressed(ButtonX);
  }

  public boolean getButtonY(){
    return getRawButton(ButtonY);
  }
  public boolean getButtonYPressed(){
    return getRawButtonPressed(ButtonY);
  }

  public boolean getButtonLeftBumper(){
    return getRawButton(ButtonLeftBumper);
  }
  public boolean getButtonLeftBumperPressed(){
    return getRawButtonPressed(ButtonLeftBumper);
  }

  public boolean getButtonRightBumper(){
    return getRawButton(ButtonRightBumper);
  }
  public boolean getButtonRightBumperPressed(){
    return getRawButtonPressed(ButtonRightBumper);
  }

  public boolean getButtonBack(){
    return getRawButton(ButtonBack);
  }
  public boolean getButtonBackPressed(){
    return getRawButtonPressed(ButtonBack);
  }

  public boolean getButtonStart(){
    return getRawButton(ButtonStart);
  }
  public boolean getButtonStartPressed(){
    return getRawButtonPressed(ButtonStart);
  }

  public boolean getButtonLeftStick(){
    return getRawButton(ButtonLeftStick);
  }
  public boolean getButtonLeftStickPressed(){
    return getRawButtonPressed(ButtonLeftStick);
  }

  public boolean getButtonRightStick(){
    return getRawButton(ButtonRightStick);
  }
  public boolean getButtonRightStickPressed(){
    return getRawButtonPressed(ButtonRightStick);
  }

}