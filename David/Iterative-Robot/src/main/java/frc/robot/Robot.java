
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends IterativeRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Joystick joy1 = new Joystick(0);

  boolean slowdown = joy1.getRawButton(1);
  double speedmod = 1;
double sensoroutput = 0;
boolean limitin;

private static final AnalogInput sensor = new AnalogInput(1);
private static final DigitalInput limit = new DigitalInput(0);

private static final WPI_TalonSRX talon1R = new WPI_TalonSRX(2);
private static final WPI_TalonSRX talon2R = new WPI_TalonSRX(4);
SpeedControllerGroup talon_R = new SpeedControllerGroup(talon1R, talon2R);

private static final WPI_TalonSRX talon1L = new WPI_TalonSRX(1);
private static final WPI_TalonSRX  talon2L = new WPI_TalonSRX(3);
SpeedControllerGroup talon_L = new SpeedControllerGroup(talon1L, talon2L);

private static final WPI_TalonSRX leftarm = new WPI_TalonSRX(5);
private static final WPI_TalonSRX rightarm = new WPI_TalonSRX(6);
private static final WPI_TalonSRX armmotor = new WPI_TalonSRX(7);

DifferentialDrive pdrive = new DifferentialDrive(talon_L, talon_R);

  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    talon_L.setInverted(true);
    talon_R.setInverted(false);

    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(640, 400);
    UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    camera2.setResolution(640, 400);
    camera1.setFPS(24);
    camera2.setFPS(24);
    pdrive.setRightSideInverted(false);
  }
  @Override
  public void robotPeriodic() 
  {
  }
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:

        break;
      case kDefaultAuto:
      default:
        break;
    }
  }
  @Override
  public void teleopPeriodic() 
  {
    sensoroutput = (int) (sensor.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Sensor Output", sensoroutput);
    limitin = limit.get();
    SmartDashboard.putBoolean("Limit Switch", limitin);
  
    if (slowdown = true)
    {
      speedmod = 1.0;
    }
    else
    {
      speedmod = 0.5;
    }

  double forwardbackward;
  double rotation;
  if (sensoroutput >= 7 & sensoroutput <= 11 )
    {
      forwardbackward = 0;
      rotation = 0;
    }
    else
    {
      forwardbackward = joy1.getRawAxis(1);
      rotation = joy1.getRawAxis(4);
    }
  pdrive.arcadeDrive(-forwardbackward * speedmod, rotation * speedmod);
  
  boolean shoot = joy1.getRawButton(2);
  boolean intake = joy1.getRawButton(3);
  boolean armup = joy1.getRawButton(4);
  boolean armdown = joy1.getRawButton(5);

  if (armup == true & armdown == false)
  {
    armmotor.set(0.7);
  }
  else if (armup == false & armdown == true)
  {
    armmotor.set(-0.7);
  }
  else
  {
    armmotor.set(0);
  }
  SmartDashboard.putNumber("Arm Motor Value", armmotor.get());
  if (shoot == true & intake == false)
  {
    leftarm.set(-1.0);
    rightarm.set(1.0);
  }
  else if (shoot == false & intake == true)
  {
    leftarm.set(1.0);
    rightarm.set(-1.0);
  }
  else
  {
    leftarm.set(0);
    rightarm.set(0);
  }
  }
  @Override
  public void testPeriodic() 
  {
  }
}
