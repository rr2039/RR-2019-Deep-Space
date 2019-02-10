package frc.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.sql.Time;

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

public class Robot_Kevin_Zuko_My_Threading extends TimedRobot 
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

  double driveXAxis;
  double driveYAxis;
  double driveRotation;
  //double driveDeadzone = 0.20; 

  double USSLout;
  double USSRout;
  double USSmod;
  double lineUS;

  double gyroangle;
  boolean lineup;
  //boolean lineupfinish;
  boolean center = false;
  
  /*boolean neverOnLine = true;
  boolean toggled = false;
  boolean center;
  int iteration = 0;
  double strafeSpeed = 1;*/

  private double centerStrafeSpeed = 0.5;
  private double centerReverseSpeed = 0.38;
  private double centerIterations = 25;
  private double tempXAxis = 0;
  private double tempYAxis = 0;
  private double tempRotation = 0;
  //Color sensor error estimate
  private int alphaFirst = 0;

  private String driveState = "normal";
  private String centerState = "start";
  private String lineupState = "start";

  Joystick joy1 = new Joystick(0);
  boolean slowdown = false;
  double slowmodifer = 1.0;

  Joystick joy2 = new Joystick(1);

  //Registries for the Color Sensor V2
  int CMD = 0x80;
  int MULTI_BYTE_BIT = 0x20;
  int ENABLE_REGISTER  = 0x00;
  int ATIME_REGISTER   = 0x01;
  int PPULSE_REGISTER  = 0x0E;

  int ID_REGISTER     = 0x12;
  int CDATA_REGISTER  = 0x14; // Clear (Alpha) register

  int PON   = 0b00000001;
  int AEN   = 0b00000010;
  int PEN   = 0b00000100;
  int WEN   = 0b00001000;
  int AIEN  = 0b00010000;
  int PIEN  = 0b00100000;

  //Some variables needed for the Color V2 Sensor
  private final double integrationTime = 10;
  private ByteBuffer buffer = ByteBuffer.allocate(10);
  /*
   * The sensor reads 68-71 on the line, 38-41 half off, and 10-12 off the line.
   * The Threshold is set below that just to be safe in case it never finds that value.
   */
  private short alpha = 0, threshold = 38;
  //private double xSpeed = 0;
  
  //Initialize the Color V2 Sensor
  I2C sensor = new I2C(I2C.Port.kOnboard, 0x39);

  //double xaxis = 0;
  //private static final boolean dashboardDriveDirection;

  private static final AnalogInput sensorL = new AnalogInput(0);
  private static final AnalogInput sensorR = new AnalogInput(1);
 
  private static final WPI_TalonSRX talonFR = new WPI_TalonSRX(1);
  private static final WPI_TalonSRX talonBR = new WPI_TalonSRX(11);
  private static final SpeedControllerGroup talon_R = new SpeedControllerGroup(talonFR, talonBR);

  private static final WPI_TalonSRX talonFL = new WPI_TalonSRX(2);
  private static final WPI_TalonSRX talonBL = new WPI_TalonSRX(3);
  private static final SpeedControllerGroup talon_L = new SpeedControllerGroup(talonFL, talonBL);

  private DifferentialDrive drive = new DifferentialDrive(talon_L,talon_R);
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
    //SmartDashboard.putBoolean("lineupfinish", lineupfinish);

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

    talonFR.configOpenloopRamp(0.25, 20);
    talonFL.configOpenloopRamp(0.25, 20);
    talonBL.configOpenloopRamp(0.25, 20);
    talonBR.configOpenloopRamp(0.25, 20);
    
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
    //alpha = readAlphaIntensity();
    SmartDashboard.putNumber("Alpha Value", alpha);
    SmartDashboard.putNumber("PoutFR", poutFR);

    SmartDashboard.putString("driveState", driveState);
    SmartDashboard.putString("centerState", centerState);

    SmartDashboard.putNumber("Alpha when first detected", alphaFirst);

  
    //normal drive state
    if (driveState.equals("normal"))
    {
      if (center && joy1.getRawButtonPressed(3))
      {
        driveState = "center";
      }
      else if (lineup && joy1.getRawButtonPressed(2))
      {
        driveState = "lineup";
      }

      else
      {
        driveNormal();
      }     
      
    }

    //center state
    else if(driveState.equals("center"))
    {
      /*
     * The robot will only center if it was not on the line previously.
     * This (neverOnLine) is reset to true if the button is released.
     * Strafe speed works at -0.5, reverse speed at 0.5, and iterations at 14 for perfect accuracy.
     * There is room for improvement for speed.
     * 
     * For strafe speed at -0.6, reverse speed at 0.38, and iterations at 25.
     * Not as accurate as the previous but could potentially be improved to the same accuracy.
     */
      if (!center || centerState.equals("stop"))
      {
        tempXAxis = 0;
        drive.arcadeDrive(0,0);
        driveState = "normal";
        centerState = "start";
      }

      else
      {
        driveCenter();
      }
    }

    //lineup state
    else if (driveState.equals("lineup"))
    {
      if (!lineup || lineupState.equals("stop"))
      {
        tempXAxis = 0;
        tempYAxis = 0;
        tempRotation = 0;
        drive.arcadeDrive(0,0);
        driveState = "normal";
        lineupState = "start";
      }

      else
      {
        driveLineup();
      }
    }

    
    
    /*
     * The robot will only center if it was not on the line previously.
     * This (neverOnLine) is reset to true if the button is released.
     * Strafe speed works at -0.5, reverse speed at 0.5, and iterations at 14 for perfect accuracy.
     * There is room for improvement for speed.
     * 
     * For strafe speed at -0.6, reverse speed at 0.38, and iterations at 25.
     * Not as accurate as the previous but could potentially be improved to the same accuracy.
     */
    /*if (center == true && toggled == false && neverOnLine == true)
    {
      toggled = true;
      strafeSpeed = 0.4;
      xaxis = 0;
      CenterRobot();
      mecdrive.driveCartesian(-0.6, 0, 0);
    }
    else if (center == true && toggled == true && neverOnLine == true)
    {
      CenterRobot();
      mecdrive.driveCartesian(-0.6, 0, 0);
    }
    else if (center == true && neverOnLine == false)
    {
      if (iteration < 25)
      {
        mecdrive.driveCartesian(0.38, 0, 0);
        iteration += 1;
      }
      else{
        xSpeed = 0;
      }
    }
    else if (center == false)
    {
      toggled = false;
      neverOnLine = true;
      iteration = 0;
      mecdrive.driveCartesian(xaxis * slowmodifer, -yaxis * slowmodifer, rotation * slowmodifer);
    }*/
      SmartDashboard.putNumber("driveXAxis", driveXAxis);
      SmartDashboard.putNumber("driveYAxis", driveYAxis);
      SmartDashboard.putNumber("driveRotation", driveRotation);
      //lineupfinish = false;
  }
    // ----------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() 
  {

  }
  /*public void USCorrection()
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
  }*/

  /*public void CenterRobot()
  {
    /*The robot strafes left until it hits the line. 
    *
    if (alpha >= threshold)
    {
      neverOnLine = false;
      xSpeed = 0.5;
    }
    else if (alpha < threshold)
    {
      xSpeed = -strafeSpeed;
    }
  }*/
  
  public void driveNormal()
  {
    drive.setDeadband(0.2);
    if (slowdown)
    {
      slowmodifer = 0.5;
    }
    else
    {
      slowmodifer = 1.0;
    }

    /*if (joy1.getRawAxis(0) > driveDeadzone || joy1.getRawAxis(0) < -driveDeadzone)
    {
      driveXAxis = joy1.getRawAxis(0);
    }
    else
    {
      driveXAxis = 0;
       }
    if (joy1.getRawAxis(1) > driveDeadzone || joy1.getRawAxis(1) < -driveDeadzone)
    {
      driveYAxis = joy1.getRawAxis(1);
    }
    else
    {
      driveYAxis = 0;
    }
    if (joy1.getRawAxis(4) > driveDeadzone || joy1.getRawAxis(4) < -driveDeadzone)
    {
      driveRotation = joy1.getRawAxis(4);
    }
    else
    {
      driveRotation = 0;
    }*/

    driveXAxis = joy1.getRawAxis(0);
    driveYAxis = joy1.getRawAxis(1);
    driveRotation = joy1.getRawAxis(4);
    drive.arcadeDrive(driveXAxis*slowmodifer, driveRotation*slowmodifer);
  }

  public void driveCenter()
  {
    if (centerState.equals("start"))
    {
      
      quickThread colorThread = new quickThread();
      colorThread.start();
      drive.setDeadband(0);
      centerState = "run";
    }
    else if (centerState.equals("run"))
    {
      ;
    }
    else if (centerState.equals("reverse"))
    {
      if (centerIterations >= 25)
      {
        centerIterations = 0;
        drive.arcadeDrive(0,0);
        centerState = "stop";
      }

      else
      {
        tempXAxis = centerReverseSpeed;
        centerIterations += 1;
      }
    }

    if (!centerState.equals("run")) drive.arcadeDrive(tempXAxis,tempRotation);
  }

  public void driveLineup()
  {
    if (lineupState.equals("start"))
      {
        if (Math.abs(USSLout - USSRout) <= 2)
        {
          drive.arcadeDrive(0,0);
          lineupState = "stop";
        }

        else
        {
          drive.setDeadband(0);
          lineUS = ((USSLout - USSRout) / 500);
          SmartDashboard.putNumber("USSmod", USSmod);

          if (USSLout > USSRout)
          {
            tempRotation = 0.175 + lineUS;
          }
          else
          {
            tempRotation = -0.175 + lineUS;
          }
        }
      }

      drive.arcadeDrive(tempXAxis,tempRotation);
  }

  public short readAlphaIntensity() {
    buffer.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);

    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }

  //Quick thread
  private class quickThread extends Thread
  {
    @Override
    public void run() {
      try
      {
        while (!isInterrupted()) 
        {
          alpha = readAlphaIntensity();
          if (alpha >= threshold)
          {
            alphaFirst = alpha;
            tempXAxis = 0;
            centerState = "reverse";
            alpha = 0;
            drive.arcadeDrive(tempXAxis,tempRotation);
            this.interrupt();
          }
    
          else
          {
            tempXAxis = -centerStrafeSpeed;
          } 
          drive.arcadeDrive(tempXAxis,tempRotation);
          Thread.sleep(2000);
        }
      } 
      catch (InterruptedException e) 
      {
        System.out.println(e.getMessage());
      }
      catch (Exception e) 
      {
        System.out.println(e.getMessage());
      }
    }
  }
}