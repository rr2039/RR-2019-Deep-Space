package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.kauailabs.navx.frc.*;

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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import com.ctre.phoenix.motorcontrol.*;

// ----------------------------------------------------------------------------------------

public class Robot extends IterativeRobot 
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

/* Allignment variables */
  boolean center;
  String driveDirection = "Left";
  double strafeSpeed = 0.6;

/* Drive motor Voltage Variables. */
  double voutFR;
  double voutFL;
  double voutBL;
  double voutBR;

/* Joystick 1 Drive variables */
  double xaxis;
  double yaxis;
  double rotation;
  double deadzone = 0.20; 
  boolean liftup;
  boolean liftdown;

/* Ultrasonic Straightening Variables */
  double USSLout;
  double USSRout;
  double lineUS;

  double gyroangle;
  boolean lineup;
  boolean lineupfinish;
  boolean gyromove;

/* Joystick 1 Control variables */
  Joystick joy1 = new Joystick(0);
  boolean slowdown;
  double slowmodifer;

  Joystick joy2 = new Joystick(1);

/* Alignment Variables */
  int CMD = 0x80;
  int MULTI_BYTE_BIT = 0x20;
  int ENABLE_REGISTER  = 0x00;
  int ATIME_REGISTER   = 0x01;
  int PPULSE_REGISTER  = 0x0E;

  int ID_REGISTER     = 0x12;
  int CDATA_REGISTER  = 0x14; 

  int PON   = 0b00000001;
  int AEN   = 0b00000010;
  int PEN   = 0b00000100;
  int WEN   = 0b00001000;
  int AIEN  = 0b00010000;
  int PIEN  = 0b00100000;

  private final double integrationTime = 10;
  private ByteBuffer buffer = ByteBuffer.allocate(10);
  private short alpha = 0, threshold = 25;
  
  I2C sensor = new I2C(I2C.Port.kOnboard, 0x39);

// Everything Else
  AHRS ahrs = new AHRS(SerialPort.Port.kUSB1);
  double ZRotation = 0;

  AnalogInput sensorL = new AnalogInput(0);
  AnalogInput sensorR = new AnalogInput(1);
 
  WPI_TalonSRX talonFR = new WPI_TalonSRX(3);
  WPI_TalonSRX talonBR = new WPI_TalonSRX(4);

  WPI_TalonSRX talonFL = new WPI_TalonSRX(1);
  WPI_TalonSRX talonBL = new WPI_TalonSRX(2);

  WPI_TalonSRX liftmotor = new WPI_TalonSRX(5);

 MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
  Compressor c = new Compressor(0);
  DoubleSolenoid Solenoid1 = new DoubleSolenoid(1, 3);
  DoubleSolenoid Solenoid2 = new DoubleSolenoid(0, 2);
// ----------------------------------------------------------------------------------------
  @Override
  public void robotInit() 
  {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    /* Initialize and Configure Cameras */
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera1.setResolution(640, 400);
    UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    camera2.setResolution(640, 400);
    camera1.setFPS(24);
    camera2.setFPS(24);

    // Allignment Preliminary Code
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    sensor.write(CMD | 0x00, PON | AEN | PEN);
    sensor.write(CMD | 0x01, (int) (256-integrationTime/2.38)); 
    sensor.write(CMD | 0x0E, 0b1111);
  }
// ----------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() 
  {
    
    SmartDashboard.putNumber("alphavalue", alpha);
    center = joy2.getRawButton(5);
    slowdown = joy1.getRawButton(1);
    lineup = joy1.getRawButton(2);
    gyromove = joy1.getRawButton(3);
    
    ZRotation = ahrs.getAngle();
    
    SmartDashboard.putBoolean("slowmode", slowdown);
    SmartDashboard.putBoolean("lineup", lineup);
    SmartDashboard.putBoolean("lineupfinish", lineupfinish);

    SmartDashboard.putNumber("Yaw", ZRotation);
    SmartDashboard.putBoolean("is connected", ahrs.isConnected());
    SmartDashboard.putNumber("XDisp", ahrs.getDisplacementX());
    SmartDashboard.putNumber("YDisp", ahrs.getDisplacementY());

    USSLout = (int) (sensorL.getAverageVoltage() * 147);
    USSRout = (int) (sensorR.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    /* Drivetrain Ramping */
    talonFR.configOpenloopRamp(0.20, 20);
    talonFL.configOpenloopRamp(0.20, 20);
    talonBL.configOpenloopRamp(0.20, 20);
    talonBR.configOpenloopRamp(0.20, 20);

    /* Deadzone Logic */
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
    /* Lift Logic */
    if (liftup == true)
    {
      liftmotor.set(0.7);
    }
    else if (liftdown == true)
    {
      liftmotor.set(-0.7);
    }
    else
    {
      liftmotor.set(0.0);
    }

    /* Pnumatics Logic */
    if(joy2.getRawButton(1))
    {
    Solenoid1.set(DoubleSolenoid.Value.kForward);
    }
    else if(joy2.getRawButton(2))
    {
    Solenoid1.set(DoubleSolenoid.Value.kReverse);
    }
    if(joy2.getRawButton(3))
    {
    Solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    else if(joy2.getRawButton(4))
    {
    Solenoid2.set(DoubleSolenoid.Value.kReverse);
    }
    

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
    if (center == true)
    {
      CenterRobot();
    }
      mecdrive.driveCartesian(xaxis * slowmodifer, -yaxis * slowmodifer, rotation * slowmodifer);
      lineupfinish = false;
  }
    // ----------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() 
  {

  }
  /* Ultrasonic Lineup Function */
   public void USCorrection()
  {
    lineUS = ((USSLout - USSRout) / 500);

    if (Math.abs(USSLout - USSRout) <= 2)
    {
      rotation = 0;
      yaxis = -0.20;
      ahrs.reset();
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
 /* Lightsensor Lineup Function */
  public void CenterRobot()
  {
    if (driveDirection == "Left" && alpha < threshold)
    {
      yaxis = -strafeSpeed;
      if (alpha > threshold)
      {
        driveDirection = "Right";
        yaxis = 0;
        if (strafeSpeed/2 <= 0.2) {strafeSpeed = strafeSpeed/2 ;};
      }
    }
    else if (driveDirection == "Right" && alpha < threshold)
    {
      yaxis = strafeSpeed;
      if (alpha > threshold)
      {
        driveDirection = "Left";
        yaxis = 0;
        if (strafeSpeed/2 >= 0.2) {strafeSpeed = strafeSpeed/2; }
      }
    }
    else
    {
      yaxis = 0;
    }
  }
  public short readAlphaIntensity() 
  {
    buffer.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);
    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }
}