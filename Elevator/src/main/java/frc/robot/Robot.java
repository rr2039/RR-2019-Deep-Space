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

public class Robot extends TimedRobot 
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

  double USSLout;
  double USSRout;
  double USSmod;
  double lineUS;

  float error;
  float yaw;
  float targetYaw;
  boolean lineup;
  boolean center = false;
  boolean driveStraightButton;
  float feedbackLoopOutput;
  float processedOutput;
  
  boolean hatchLevel1Button;
  boolean hatchLevel2Button;
  boolean hatchLevel3Button;
  boolean cargoLevel1Button;
  boolean cargoLevel2Button;
  boolean cargoLevel3Button;

  //These variables determine the state of the pickup system.
  boolean startingPositionButton;
  boolean hatchingPositionButton;
  boolean hatchFloorPositionButton;
  boolean cargoPositionButton;
  
  private double centerStrafeSpeed = 0.7;
  private double centerReverseSpeed = 0.4;
  private double centerReverseSpeed2 = 0.38;
  private int centerIterations = 10;//how many iterations to reverse
  private int centerIterations2 = 10;
  private int centerIterationsCount = 0;//the count of iterations to reverse
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
  AHRS ahrs;

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
  
  //Initialize the Color V2 Sensor
  I2C sensor = new I2C(I2C.Port.kOnboard, 0x39);

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

    //Initialize NavX-Micro
    try
    {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
    }
    catch (RuntimeException ex)
    {
      DriverStation.reportError("Error in instantiating navx-Micro: " + ex.getMessage(), true);
    }
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
    poutFR = talonFR.getMotorOutputPercent();
    alpha = readAlphaIntensity();
    SmartDashboard.putNumber("Alpha Value", alpha);
    SmartDashboard.putNumber("PoutFR", poutFR);

    SmartDashboard.putString("driveState", driveState);
    SmartDashboard.putString("centerState", centerState);

    SmartDashboard.putNumber("Alpha when first detected", alphaFirst);
    
    SmartDashboard.putNumber("driveXAxis", driveXAxis);
    SmartDashboard.putNumber("driveYAxis", driveYAxis);
    SmartDashboard.putNumber("driveRotation", driveRotation);  
    hatchLevel1Button = SmartDashboard.getBoolean("Hatch Level 1", false);
    hatchLevel2Button = SmartDashboard.getBoolean("Hatch Level 2", false);
    hatchLevel3Button =  SmartDashboard.getBoolean("Hatch Level 3", false);
    cargoLevel1Button = SmartDashboard.getBoolean("Cargo Level 1", false);
    cargoLevel2Button = SmartDashboard.getBoolean("Cargo Level 2", false);
    cargoLevel3Button = SmartDashboard.getBoolean("Cargo Level 3", false);
    
    startingPositionButton = SmartDashboard.getBoolean("Starting Positoin", false);
    hatchingPositionButton = SmartDashboard.getBoolean("Hatching Position", false);
    hatchFloorPositionButton = SmartDashboard.getBoolean("Hatch Floor Position", false);
    cargoPositionButton = SmartDashboard.getBoolean("Cargo Position", false);

    switch (driveState)
    {
      //normal drive state
      case "normal":
      {
        if (center && joy1.getRawButtonPressed(3))
        {
          driveState = "center";
        }
        else if (lineup && joy1.getRawButtonPressed(2))
        {
          driveState = "lineup";
        }
        else if (driveStraightButton)
        {
          driveState = "driveStraight";
        }
        else if (hatchLevel1Button)
        {
          driveState = "hatchLevel1";
        }
        else if (hatchLevel2Button)
        {
          driveState = "hatchLevel2";
        }
        else if (hatchLevel3Button)
        {
          driveState = "hatchLevel3";
        }
        else if (cargoLevel1Button)
        {
          driveState = "cargoLevel1";
        }
        else if (startingPositionButton)
        {
          driveState = "startingPosition";
        }
        else if (hatchingPositionButton)
        {
          driveState = "hatchingPosition";
        }
        else if (hatchFloorPositionButton)
        {
          driveState = "hatchingFloorPosition";
        }
        else if (cargoPositionButton)
        {
          driveState = "cargoPosition";
        }
  
        else
        {
          driveNormal();
        }

        break;
      }

      //center state
      case "center":
      {
        if (!center || centerState.equals("stop"))
        {
          tempXAxis = 0;
          mecdrive.driveCartesian(0,0,0);
          driveState = "normal";
          centerState = "start";
        }
  
        else
        {
          driveCenter();
        }

        break;
      }

      //lineup state
      case "lineup":
      {
        if (!lineup || lineupState.equals("stop"))
        {
          tempXAxis = 0;
          tempYAxis = 0;
          tempRotation = 0;
          mecdrive.driveCartesian(0,0,0);
          driveState = "normal";
          lineupState = "start";
        }
  
        else
        {
          driveLineup();
        }

        break;
      }//end of lineup

      case "driveStraight":
      {
        // Drive Straight using gyro.
        driveStraight();
        
      }
      // Pickup system states

      case "startingStartingPosition":
      {
        // Pickup system is inside frame.
        driveNormal();
      }

      case "hatchingPosition":
      {
        // Position for hatching and picking up from loading station
        driveNormal();
      }

      case "hatchFloorPosition":
      {
        // Position for picking hatches up from the floor
        driveNormal();
      }

      case "cargoPosition":
      {
        // Position for cargo
        driveNormal();
      }
      //Elevator states
      
      case "hatchLevel1":
      {
        //move until level 1
        driveNormal();
      }
      case "hatchLevel2":
      {
        //move until level 2
        driveNormal();
      }
      case "hatchLevel3":
      {
        //move until next level 3
        driveNormal();
      }
     
      case "cargoLevel1":
      {
        //move until level 1
        driveNormal();
      }
      case "cargoLevel2":
      {
        //move until level 2
        driveNormal();
      }
      case "cargoLevel3":
      {
        //move until next level 3
        driveNormal();
      }
    }//end of switch

  }//end of teleop
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

  public void driveNormal()
  {
    mecdrive.setDeadband(0.2);
    if (slowdown)
    {
      slowmodifer = 0.5;
    }
    else
    {
      slowmodifer = 1.0;
    }

    driveXAxis = joy1.getRawAxis(0);
    driveYAxis = joy1.getRawAxis(1);
    driveRotation = joy1.getRawAxis(4);
    mecdrive.driveCartesian(driveXAxis * slowmodifer, -driveYAxis * slowmodifer, driveRotation * slowmodifer);
  }

  public void driveCenter()
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

    switch (centerState)
    {
      //start centering
      case "start":
      {
        if (alpha >= threshold)
        {
          alphaFirst = alpha;
          centerState = "reverse";
        }
  
        else
        {
          mecdrive.setDeadband(0);
          tempXAxis = -centerStrafeSpeed;
        }

        break;
      }

      //reverse state
      case "reverse":
      {
        if (centerIterationsCount >= centerIterations)
        {
          centerIterationsCount = 0;
          mecdrive.driveCartesian(0,0,0);
          centerState = "come_back";
        }
  
        else
        {
          tempXAxis = centerReverseSpeed;
          centerIterationsCount += 1;
        }

        break;
      }//end of reverse

      //come back until line again
      case "come_back":
      {
        if (alpha >= threshold)
        {
          centerState = "reverse2";
        }
  
        else
        {
          tempXAxis = centerReverseSpeed;
        }

        break;
      }

      //second reverse
      case "reverse2":
      {
        if (centerIterationsCount >= centerIterations2)
        {
          centerIterationsCount = 0;
          mecdrive.driveCartesian(0,0,0);
          centerState = "stop";
        }
  
        else
        {
          tempXAxis = -centerReverseSpeed2;
          centerIterationsCount += 1;
        }

        break;
      }
    }//end of switch

    mecdrive.driveCartesian(tempXAxis,tempYAxis,tempRotation);
  }//end of driveCenter()

  public void driveLineup()
  {
    switch (lineupState)
    {
      //start lineup
      case "start":
      {
        if (Math.abs(USSLout - USSRout) <= 2)
        {
          mecdrive.driveCartesian(0,0,0);
          lineupState = "stop";
        }

        else
        {
          mecdrive.driveCartesian(0,0,0);
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

        break;
      }
    }//end of switch

    mecdrive.driveCartesian(tempXAxis,tempYAxis,tempRotation);
  }//end of driveLineup()

  public short readAlphaIntensity() {
    buffer.clear();
    sensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);

    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }

  public float feedbackLoop(float target, float error)
  {
    return 15;
  }

  public float processOutput(float output)
  {
    output = output/1;
    if (output > 1)
    {
      return 1;
    }
    else if (output < -1)
    {
      return -1;
    }
    else if (Math.abs(output) < 0.4)
    {
      return (float) 0.4;
    }
    else
    {
      return output;
    }
  }

  public void driveStraight()
  {
    if (driveStraightButton == true)
    {
      ahrs.reset();
      yaw = ahrs.getYaw();
      targetYaw = yaw;
    }
    else
    {
      yaw = ahrs.getYaw();
      error = Math.abs(0-yaw);
      feedbackLoopOutput = feedbackLoop(targetYaw, error);
      processedOutput = processOutput(feedbackLoopOutput); 
      mecdrive.driveCartesian(0.5, 0, processedOutput);
    }
  }

  public void goToLevel(int target_level)
  {
    //move to target level
  }
}