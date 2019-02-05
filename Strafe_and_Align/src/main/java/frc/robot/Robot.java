package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
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
import edu.wpi.first.wpilibj.I2C;

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
 
  boolean center;
  String driveDirection = "Left";
  double strafeSpeed = 0.4;

  Joystick joy1 = new Joystick(0);
  boolean slowdown;
  double slowmodifer;

  Joystick joy2 = new Joystick(1);

  //Registries for the Color Sensor V2
  protected final static int CMD = 0x80;
  protected final static int MULTI_BYTE_BIT = 0x20;
  protected final static int ENABLE_REGISTER  = 0x00;
  protected final static int ATIME_REGISTER   = 0x01;
  protected final static int PPULSE_REGISTER  = 0x0E;

  protected final static int ID_REGISTER     = 0x12;
  protected final static int CDATA_REGISTER  = 0x14; // Clear (Alpha) register

  protected final static int PON   = 0b00000001;
  protected final static int AEN   = 0b00000010;
  protected final static int PEN   = 0b00000100;
  protected final static int WEN   = 0b00001000;
  protected final static int AIEN  = 0b00010000;
  protected final static int PIEN  = 0b00100000;

  //Some variables needed for the Color V2 Sensor
  private final double integrationTime = 10;
  private ByteBuffer buffer = ByteBuffer.allocate(10);
  //The sensor reads 25-38 on the line, 12 half off, and 6-8 off the line
  private short alpha = 0, threshold = 50;
  
  //Initialize the Color V2 Sensor
  I2C sensor = new I2C(I2C.Port.kOnboard, 0x39);

  //double xaxis = 0;
  private static final double minimumStrafeSpeed = 0.2;
  //private static final boolean dashboardDriveDirection;

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

    //Some initializing for Color V2 Sensor.
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    sensor.write(CMD | 0x00, PON | AEN | PEN);
    sensor.write(CMD | 0x01, (int) (256-integrationTime/2.38)); //configures the integration time (time for updating color data)
    sensor.write(CMD | 0x0E, 0b1111);
  }
// ----------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() 
  {
  
    slowdown = joy1.getRawButton(1);
    lineup = joy1.getRawButton(2);
    center = joy1.getRawButton(3);
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
    /*if (driveDirection == "Left")
    {
      dashboardDriveDirection == True;
    }
    else
    {
      dashboardDriveDirection == False;
     }*/
    poutFR = talonFR.getMotorOutputPercent();
    alpha = readAlphaIntensity();
    SmartDashboard.putNumber("Alpha Value", alpha);
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

    if (center == true)
    {
      CenterRobot();
    }
      mecdrive.driveCartesian(xaxis * slowmodifer, -yaxis * slowmodifer, rotation * slowmodifer);
      SmartDashboard.putNumber("xaxis", xaxis);
      SmartDashboard.putNumber("yaxis", yaxis);
      SmartDashboard.putNumber("rotation", rotation);
      SmartDashboard.putString("Drive Direction", driveDirection);
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

  public void CenterRobot()
  {
    /*The robot strafes left, if it hits the line it stops and inverts the driving direction. 
    */
    if (driveDirection == "Left" && alpha < threshold)
    {
      xaxis = -strafeSpeed;
    }
    else if (driveDirection == "Right" && alpha < threshold)
    {
      xaxis = strafeSpeed;
    }
    else
    {
      //Do not straffe because the robot is on the line.
      xaxis = 0;
      if (strafeSpeed/2 >= minimumStrafeSpeed) {strafeSpeed = strafeSpeed/2; }
      if (driveDirection == "Left")
      {
        driveDirection = "Right";
      }
      else
      {
        driveDirection = "Left";
      }

    }
  }
  
  public short readAlphaIntensity() {
    buffer.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);

    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }
}