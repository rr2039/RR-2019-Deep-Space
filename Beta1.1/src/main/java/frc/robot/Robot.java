package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogOutput;

import com.ctre.phoenix.motorcontrol.*;

// ----------------------------------------------------------------------------------------

public class Robot extends IterativeRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double voutFR;
  double voutFL;
  double voutBL;
  double voutBR;

  double poutFR;
  double poutBR;
  double poutBL;
  double poutFL;

  double xaxis;
  double yaxis;
  double rotation;
  double deadzone = 0.20; 

  double USSLout;
  double USSRout;
  double USSmod;
  double lineUS;

  double gyroangle;
  boolean lineup;
  boolean lineupfinish;

  Joystick joy1 = new Joystick(0);
  boolean slowdown;
  double slowmodifer;

  Joystick joy2 = new Joystick(1);

  private static final AnalogInput sensorL = new AnalogInput(0);
  private static final AnalogInput sensorR = new AnalogInput(1);
 
  private static final WPI_TalonSRX talonFR = new WPI_TalonSRX(3);
  private static final WPI_TalonSRX talonBR = new WPI_TalonSRX(4);
  private static final SpeedControllerGroup talon_R = new SpeedControllerGroup(talonFR, talonBR);

  private static final WPI_TalonSRX talonFL = new WPI_TalonSRX(1);
  private static final WPI_TalonSRX talonBL = new WPI_TalonSRX(2);
  private static final SpeedControllerGroup talon_L = new SpeedControllerGroup(talonFL, talonBL);

 MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
// ----------------------------------------------------------------------------------------
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(640, 400);
    UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    camera2.setResolution(640, 400);
    camera1.setFPS(24);
    camera2.setFPS(24);
  }
// ----------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() 
  {
  
    slowdown = joy1.getRawButton(1);
    lineup = joy1.getRawButton(2);
    SmartDashboard.putBoolean("slowmode", slowdown);
    SmartDashboard.putBoolean("lineup", lineup);
    SmartDashboard.putBoolean("lineupfinish", lineupfinish);

    USSLout = (int) (sensorL.getAverageVoltage() * 147);
    USSRout = (int) (sensorR.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    voutFR = talonFR.getMotorOutputVoltage();
    voutFL = talonFL.getMotorOutputVoltage();
    voutBL = talonBL.getMotorOutputVoltage();
    voutBR = talonBR.getMotorOutputVoltage();
    SmartDashboard.putNumber("Volt FR", voutFR);
    SmartDashboard.putNumber("Volt FL", voutFL);
    SmartDashboard.putNumber("Volt BL", voutBL);
    SmartDashboard.putNumber("Volt BR", voutBR);

    talonFR.getMotorOutputPercent();

    talonFR.configOpenloopRamp(0.25, 20);
    talonFL.configOpenloopRamp(0.25, 20);
    talonBL.configOpenloopRamp(0.25, 20);
    talonBR.configOpenloopRamp(0.25, 20);

    if (joy1.getRawAxis(0) > deadzone || joy1.getRawAxis(0) < -deadzone)
    {
      xaxis = joy1.getRawAxis(0);
    }
    else
    {
      xaxis = 0;
    }
    if (joy1.getRawAxis(1) > deadzone || joy1.getRawAxis(1) < -deadzone)
    {
      yaxis = joy1.getRawAxis(1);
    }
    else
    {
      yaxis = 0;
    }
    if (joy1.getRawAxis(4) > deadzone || joy1.getRawAxis(4) < -deadzone)
    {
      rotation = joy1.getRawAxis(4);
    }
    else
    {
      rotation = 0;
    }
  }
  // ----------------------------------------------------------------------------------------
  @Override
  public void autonomousInit() 
  {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }
  // ----------------------------------------------------------------------------------------
  @Override
  public void autonomousPeriodic() 
  {
    switch (m_autoSelected) 
    {
      case kCustomAuto:

        break;
      case kDefaultAuto:
      default:

        break;
    }
  }
   // ----------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic() 
  {
    poutFR = talonFR.getMotorOutputPercent();
    SmartDashboard.putNumber("PoutFR", poutFR);
    if (slowdown == true)
    {
      slowmodifer = 0.5;
    }
    else if (slowdown == false)
    {
      slowmodifer = 1.0;
    }
    
    if (lineup == true)
    {
      USCorrection();
    }
      mecdrive.driveCartesian(xaxis * slowmodifer, -yaxis * slowmodifer, rotation * slowmodifer);
      lineupfinish = false;
  }
    // ----------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() 
  {

  }
   public void USCorrection()
  {
    lineUS = ((USSLout - USSRout) / 500);
    SmartDashboard.putNumber("USSmod", USSmod);

    if (Math.abs(USSLout - USSRout) <= 2)
    {
      rotation = 0;
      yaxis = -0.20;
    }
    else if (USSLout > USSRout)
    {
     rotation = 0.175 + lineUS;
    }
    else if (USSLout < USSRout)
    {
      rotation = -0.175 + lineUS;
    }
  } 
}