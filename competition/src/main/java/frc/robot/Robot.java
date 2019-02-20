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

public class Robot extends TimedRobot
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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

/* Manual drive variables */
  double driveXAxis;
  double driveYAxis;
  double driveRotation;

/* Ultrasonic Straightening Variables */
  double USSLout = 0;
  double USSRout = 0;
  double lineUS;

  /* Alignment Variables */
  double gyroangle;
  boolean lineup;
  boolean gyromove;
  boolean center = false;
  private double centerStrafeSpeed = 0.7;
  private double centerReverseSpeed = 0.4;
  private double centerReverseSpeed2 = 0.38;
  private int centerIterations = 10;//how many iterations to reverse
  private int centerIterations2 = 10;
  private int centerIterationsCount = 0;//the count of iterations to reverse
  //Color sensor error estimate
  private int alphaFirst = 0;

  /* Automated Drive Variables */
  private double tempXAxis = 0;
  private double tempYAxis = 0;
  private double tempRotation = 0;

  /* Robot States */
  private String driveState = "normal";
  private String operatorState = "idle";
  private String centerState = "start";
  private String lineupState = "start";
  private String centerDirection = "left";
  private int gyroDirection = 4;//facing front

  /* Operator Buttons */
  boolean hatchLevel1Button;
  boolean hatchLevel2Button;
  boolean hatchLevel3Button;
  boolean cargoLevel1Button;
  boolean cargoLevel2Button;
  boolean cargoLevel3Button;
  boolean disableSafetiesButton;

  //These variables determine the state of the pickup system.
  boolean startingPositionButton;
  boolean hatchingPositionButton;
  boolean hatchFloorPositionButton;
  boolean cargoPositionButton;

  /* Constants for the encoder values for required positions */

  //Starting Positions
  final double liftStartingPosition = 0;
  final double wristStartingPosition = 0;
  
  //Hatching pickup positions
  final double liftHatchingPosition = 0;
  final double wristHatchingPosition = 0;

  //Hatch pickup from floor positions
  final double liftHatchingFloorPosition = 0;
  final double wristHatchingFloorPosition = 0;
  
  //Cargo pickup positions
  final double liftCargoPickupPosition = 0;
  final double wristCargoPickupPosition = 0;
  
  //Hatch level 1 positions
  final double liftHatchLevel1_Position = 0;
  final double wristHatchLevel1_Position = 0;

  //Hatch level 2 positions
  final double liftHatchLevel2_Position = 0;
  final double wristHatchLevel2_Position = 0;
  
  //hatch level 3 positions
  final double liftHatchLevel3_Position = 0;
  final double wristHatchLevel3_Position = 0;

  //cargo level 1 positions
  final double liftCargoLevel1_Position = 0;
  final double wristCargoLevel1_Position = 0;

  //Cargo level 2 positions
  final double liftCargoLevel2_Position = 0;
  final double wristCargoLevel2_Position = 0;

  //Cargo level 3 positions
  final double liftCargoLevel3_Position = 0;
  final double wristCargoLevel3_Position = 0;

  /* Limit Switches */
  boolean liftLimitSwitch = false;
  boolean wristLimitSwitch = false;

/* Joystick 1 Control variables */
  Joystick joy1 = new Joystick(0);
  boolean slowdown = false;
  double slowmodifer = 1.0;
  boolean clear = false;

  Joystick joy2 = new Joystick(1);

/* Color Sensor API Variables */
  final int CMD = 0x80;
  final int MULTI_BYTE_BIT   = 0x20;
  final int ENABLE_REGISTER  = 0x00;
  final int ATIME_REGISTER   = 0x01;
  final int PPULSE_REGISTER  = 0x0E;

  final int ID_REGISTER     = 0x12;
  final int CDATA_REGISTER  = 0x14;

  final int PON   = 0b00000001;
  final int AEN   = 0b00000010;
  final int PEN   = 0b00000100;
  final int WEN   = 0b00001000;
  final int AIEN  = 0b00010000;
  final int PIEN  = 0b00100000;

  private final double integrationTime = 10;
  private final short threshold = 25;
  private ByteBuffer buffer = ByteBuffer.allocate(10);
  private short alpha = 0; 

  I2C colorSensor = new I2C(I2C.Port.kOnboard, 0x39);

// Everything Else
  AHRS ahrs = new AHRS(SerialPort.Port.kUSB1);
  double ZRotation = 0;

  AnalogInput sensorL = new AnalogInput(0);
  AnalogInput sensorR = new AnalogInput(1);

  WPI_TalonSRX talonFR = new WPI_TalonSRX(2);
  WPI_TalonSRX talonBR = new WPI_TalonSRX(3);

  WPI_TalonSRX talonFL = new WPI_TalonSRX(15);
  WPI_TalonSRX talonBL = new WPI_TalonSRX(13);

  WPI_TalonSRX liftMotor = new WPI_TalonSRX(14);
  WPI_TalonSRX wristMotor = new WPI_TalonSRX(9);

  WPI_TalonSRX climbL = new WPI_TalonSRX(12);
  WPI_TalonSRX climbR = new WPI_TalonSRX(1);

  WPI_TalonSRX WclimbL = new WPI_TalonSRX(8);
  WPI_TalonSRX WclimbR = new WPI_TalonSRX(7);

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
  
  Compressor compressor = new Compressor(0);
  Solenoid ejectorSolenoid = new Solenoid(4);
  //DoubleSolenoid Solenoid2 = new DoubleSolenoid(0, 2);
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

    // Color Sensor Startup Code
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    colorSensor.write(CMD | 0x00, PON | AEN | PEN);
    colorSensor.write(CMD | 0x01, (int) (256-integrationTime/2.38));
    colorSensor.write(CMD | 0x0E, 0b1111);

 /* 
    motor.setSensorPhase(true);
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motor.config_kP(0, 0.01, 30);
    motor.config_kI(0, 0.25, 30);
    motor.config_kD(0, 0.015, 30);
    motor.config_kF(0, 0.1, 30);
    motor.configClosedLoopPeriod(0, 1, 30);
    motor.configClearPositionOnQuadIdx(clear, 20);
  */

    liftMotor.setSensorPhase(true);
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.config_kP(0, 0.01, 30);
    liftMotor.config_kI(0, 0.25, 30);
    liftMotor.config_kD(0, 0.015, 30);
    liftMotor.config_kF(0, 0.1, 30);
    liftMotor.configClosedLoopPeriod(0, 1, 30);
    liftMotor.configClearPositionOnQuadIdx(clear, 20);

   //NavX
   try
   {
     ahrs = new AHRS(SerialPort.Port.kUSB1);
   }
   catch(RuntimeException ex)
   {
     DriverStation.reportError("NavX Setup Error" + ex.getMessage(),true);
   }
  }
// ----------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic()
  {
    if (joy2.getRawButton(1) == true)
    {
      climbL.set(-1.0);
      climbR.set(-1.0);
    }
    else if (joy2.getRawButton(4) == true)
    {
      climbL.set(1.0);
      climbR.set(1.0);
    }
    else if (joy2.getRawButton(2) == true)
    {
      WclimbL.set(joy2.getRawAxis(2));
      WclimbR.set(joy2.getRawAxis(3));
    }
    else if (joy2.getRawButton(8) == true)
    {
      climbL.set(joy2.getRawAxis(2));
      climbR.set(joy2.getRawAxis(3));
    }
    else if (joy2.getRawButton(7) == true)
    {
      climbL.set(- joy2.getRawAxis(2));
      climbR.set(- joy2.getRawAxis(3));
    }
    else
    {
      climbL.set(0);
      climbR.set(0);
      WclimbL.set(0);
      WclimbR.set(0);
    }
  
    if (joy2.getRawButton(3))
    {
      ejectorSolenoid.set(true);
    }
    else
    {
      ejectorSolenoid.set(false);
    }
    
    SmartDashboard.putNumber("alphavalue", alpha);
    center = joy2.getRawButton(5);
    //slowdown = joy1.getRawButton(1);
    lineup = joy1.getRawButton(2);
    gyromove = joy1.getRawButton(3);
    liftMotor.set(joy2.getRawAxis(1));
    wristMotor.set(joy2.getRawAxis(5));

    ZRotation = ahrs.getAngle();

    SmartDashboard.putBoolean("slowmode", slowdown);
    SmartDashboard.putBoolean("lineup", lineup);

    SmartDashboard.putNumber("Yaw", ZRotation);
    SmartDashboard.putBoolean("is connected", ahrs.isConnected());
    SmartDashboard.putNumber("XDisp", ahrs.getDisplacementX());
    SmartDashboard.putNumber("YDisp", ahrs.getDisplacementY());

    USSLout = (int) (sensorL.getAverageVoltage() * 147);
    USSRout = (int) (sensorR.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());


    /* Drivetrain Ramping */
    talonFR.configOpenloopRamp(0.25, 20);
    talonFL.configOpenloopRamp(0.25, 20);
    talonBL.configOpenloopRamp(0.25, 20);
    talonBR.configOpenloopRamp(0.25, 20);

    /* Deadzone Logic */
    //Potentially replace with setDeadBand()
    mecdrive.setDeadband(0.2);
   /* if (joy2.getRawAxis(0) > deadzone || joy2.getRawAxis(0) < -deadzone)
    {
      xaxis = joy2.getRawAxis(0);
    }
    else
    {
      xaxis = 0;
    }
    if (joy2.getRawAxis(1) > deadzone || joy2.getRawAxis(1) < -deadzone)
    {
      yaxis = joy2.getRawAxis(1);
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
    } */

    /*
    position = motor.getSelectedSensorPosition();
    velocity = motor.getSelectedSensorVelocity();
    */

    alpha = readAlphaIntensity();
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
    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());
    /* Pnumatics Logic */
    /* if(joy2.getRawButton(1))
    {
    ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else if(joy2.getRawButton(2))
    {
    ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    if(joy2.getRawButton(3))
    {
    Solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    else if(joy2.getRawButton(4))
    {
    Solenoid2.set(DoubleSolenoid.Value.kReverse);
    } */

     //drive state switcher
     if (driveState.equals("center"))
     {
       if (!center || centerState.equals("stop"))
       {
         tempXAxis = 0;
         tempYAxis = 0;
         tempRotation = 0;
         mecdrive.driveCartesian(0,0,0);
         lineupState = "start";
         centerState = "start";
         driveState = "normal";
       }
     }
     else if (driveState.equals("lineup"))
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
     }
     //no exit conditions and operations
    else
    {
      if (joy1.getRawButtonPressed(3))//X
      {
        driveState = "center";
        centerDirection = "left";
      }
      else if (joy1.getRawButtonPressed(2))//B
      {
        driveState = "center";
        centerDirection = "right";
      }
      else if (joy1.getRawButtonPressed(4))//Y
      {
        driveState = "lineup";
      }
      /*
      placeholders
      */
      else if (joy1.getRawButtonPressed(8))//A, placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 1;//back right rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 2;//right rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 3;//front right rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 4;//front
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 5;//front left rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 6;//left rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 7;//back left rocket
      }
      else if (false)//placeholder for potential touch screen control
      {
        driveState = "fixedOrientation";
        gyroDirection = 8;//loading station
      }
    }

    //Operator State Switcher
    if (joy1.getRawAxis(0) > 0.2 | joy1.getRawAxis(1) > 0.2 | joy1.getRawAxis(2) > 0.2 | joy1.getRawAxis(3) > 0.2)
    {
      operatorState = "manual";
    }
    else if (hatchLevel1Button)
    {
      operatorState = "hatchLevel1";
    }
    else if (hatchLevel2Button)
    {
      operatorState = "hatchLevel2";
    }
    else if (hatchLevel3Button)
    {
      operatorState = "hatchLevel3";
    }
    else if (cargoLevel1Button)
    {
      operatorState = "cargoLevel1";
    }
    else if (startingPositionButton)
    {
      operatorState = "startingPosition";
    }
    else if (hatchingPositionButton)
    {
      operatorState = "hatchPickupPosition";
    }
    else if (hatchFloorPositionButton)
    {
      operatorState = "hatchingFloorPickupPosition";
    }
    else if (cargoPositionButton)
    {
      operatorState = "cargoPickupPosition";
    }

    switch (driveState)
    {
      //normal drive state
      case "normal":
      {
        driveNormal();
        break;
      }

      //center state
      case "center":
      {
        driveCenter();
        break;
      }

      //lineup state
      case "lineup":
      {
        driveLineup();
        break;
      }//end of lineup

      //gyro fixed orientation state
      case "fixedOrientation":
      {
        driveFixedOrientation();
        break;
      }
    }//end of switch

    switch (operatorState)
    {
      case "manual":
      {
        //This may need to change to a different encoder position
        if ((liftMotor.getSelectedSensorPosition() == liftHatchLevel3_Position | liftLimitSwitch == false) && disableSafetiesButton == true)
        {
          liftMotor.set(ControlMode.PercentOutput, joy1.getRawAxis(0));
        }
        else
        {
          liftMotor.set(ControlMode.PercentOutput, 0);
        }
        if (wristLimitSwitch == false | wristMotor.getSelectedSensorPosition() == wristHatchingFloorPosition)
        {
          wristMotor.set(ControlMode.PercentOutput, joy1.getRawAxis(1));
        }
        else
        {
          wristMotor.set(ControlMode.PercentOutput, 0);
        }
      }
      case "hatchPickupPosition":
      {
        // Position for hatching and picking up from loading station
        liftMotor.set(ControlMode.Position, liftHatchingPosition);
        wristMotor.set(ControlMode.Position, wristHatchingPosition);
      }
      case "hatchPickupFloorPosition":
      {
        // Position for picking hatches up from the floor
        liftMotor.set(ControlMode.Position, liftHatchingFloorPosition);
        wristMotor.set(ControlMode.Position, wristHatchingFloorPosition);
      }
      case "cargoPickupPosition":
      {
        // Position for cargo
        liftMotor.set(ControlMode.Position, liftCargoPickupPosition);
        wristMotor.set(ControlMode.Position, wristCargoPickupPosition);
      }
      //Elevator states
      case "hatchLevel1":
      {
        //move until level 1 (1 ft. 7 in.)
        liftMotor.set(ControlMode.Position, liftHatchLevel1_Position);
        wristMotor.set(ControlMode.Position, wristHatchLevel1_Position);
      }
      case "hatchLevel2":
      {
        //move until level 2 (3 ft. 11 in.)
        liftMotor.set(ControlMode.Position, liftHatchLevel2_Position);
        wristMotor.set(ControlMode.Position, wristHatchLevel2_Position);
      }
      case "hatchLevel3":
      {
        //move until next level 3 (5 ft. 15 in.)
        liftMotor.set(ControlMode.Position, liftHatchLevel3_Position);
        wristMotor.set(ControlMode.Position, wristHatchLevel3_Position);
      }
      case "cargoLevel1":
      {
        //move until level 1 (2 ft. 3.5 in.)
        liftMotor.set(ControlMode.Position, liftCargoLevel1_Position);
        wristMotor.set(ControlMode.Position, wristCargoLevel1_Position);
      }
      case "cargoLevel2":
      {
        //move until level 2 (4 ft. 7.5 in)
        liftMotor.set(ControlMode.Position, liftCargoLevel2_Position);
        wristMotor.set(ControlMode.Position, wristCargoLevel2_Position);
      }
      case "cargoLevel3":
      {
        //move until next level 3 (6 ft. 11.5 in)
        liftMotor.set(ControlMode.Position, liftCargoLevel3_Position);
        wristMotor.set(ControlMode.Position, wristCargoLevel3_Position);
      }
    }
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
 
  public short readAlphaIntensity()
  {
    buffer.clear();
    colorSensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);
    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }

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

  public void driveFixedOrientation()
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
    double error = 0;
    double angle = 0;
    double heading = ahrs.getAngle();
    double kp = 0.03;
    if (gyroDirection == 1)
    {
      angle = 61.25;
    }
    else if (gyroDirection == 2)
    {
      angle = 0;
    }
    else if (gyroDirection == 3)
    {
      angle = 298.75;
    }
    else if (gyroDirection == 4)
    {
      angle = 90;
    }
    else if (gyroDirection == 5)
    {
      angle = 241.25;
    }
    else if (gyroDirection == 6)
    {
      angle = 180;
    }
    else if (gyroDirection == 7)
    {
      angle = 118.75;
    }
    else if (gyroDirection == 8)
    {
      angle = 270;
    }

    error = heading - angle;
    while (error >= 180)
    {
      error -= 360;
    }
    while (error < -180)//Must be < not <=
    {
      error += 360;
    }
    mecdrive.driveCartesian(driveXAxis * slowmodifer, -driveYAxis * slowmodifer, (0.2 + error*kp) * slowmodifer, error);
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
          if (centerDirection.equals("left"))
          {
            tempXAxis = -centerStrafeSpeed;
          }
          else if (centerDirection.equals("right"))
          {
            tempXAxis = centerStrafeSpeed;
          }
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
          if (centerDirection.equals("left"))
          {
            tempXAxis = centerReverseSpeed;
          }
          else if (centerDirection.equals("right"))
          {
            tempXAxis = -centerReverseSpeed;
          }
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
          if (centerDirection.equals("left"))
          {
            tempXAxis = centerReverseSpeed;
          }
          else if (centerDirection.equals("right"))
          {
            tempXAxis = -centerReverseSpeed;
          }
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
          centerState = "lineup";
        }
  
        else
        {
          if (centerDirection.equals("left"))
          {
            tempXAxis = -centerReverseSpeed2;
          }
          else if (centerDirection.equals("right"))
          {
            tempXAxis = centerReverseSpeed2;
          }
          centerIterationsCount += 1;
        }

        break;
      }

      //lineup using ultrasonic sensors
      case "lineup":
      {
        if (lineupState.equals("stop"))
        {
          tempXAxis = 0;
          tempYAxis = 0;
          tempRotation = 0;
          mecdrive.driveCartesian(0,0,0); 
          centerState = "drive_in";
          lineupState = "start";
        }
      }

      //lineup using ultrasonic sensors
      case "drive_in":
      {
        if (USSLout <= 11)
        {
          mecdrive.driveCartesian(0,0,0);
          centerState = "stop";
        }

        else
        {
          mecdrive.driveCartesian(0,0.4,0);
        }
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
          lineUS = ((USSLout - USSRout) / 500);

          if (USSLout > USSRout)
          {
            tempRotation = 0.175 + lineUS;
          }
          else
          {
            tempRotation = -0.175 + lineUS;
          }

          mecdrive.driveCartesian(0,0,tempRotation);
        }

        break;
      }
    }//end of switch

    mecdrive.driveCartesian(tempXAxis,tempYAxis,tempRotation);
  }//end of driveLineup()
}