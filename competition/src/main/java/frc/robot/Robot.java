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

import javax.lang.model.util.ElementScanner6;

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
  double USSLout;
  double USSRout;
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
  boolean hatchingPositionButton; //This is the same as hatch level 1
  boolean hatchFloorPositionButton;
  boolean cargoPositionButton;

  /* Limit Switches */
  boolean liftLimitSwitch = false;
  boolean wristLimitSwitch = false;

/* Joystick 1 Control variables */
  Joystick driveJoy = new Joystick(0);
  boolean slowdown = false;
  double slowmodifer = 1.0;
  boolean clear = false;

  Joystick operatorJoy = new Joystick(1);

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
  //Instantiate the navx-mxp
  AHRS ahrs = new AHRS(SerialPort.Port.kUSB1);
  double ZRotation = 0;

  AnalogInput leftUltrasonic = new AnalogInput(0);
  AnalogInput LeftUltrasonic = new AnalogInput(1);

  WPI_TalonSRX talonFR = new WPI_TalonSRX(2);
  WPI_TalonSRX talonBR = new WPI_TalonSRX(3);

  WPI_TalonSRX talonFL = new WPI_TalonSRX(15);
  WPI_TalonSRX talonBL = new WPI_TalonSRX(13);

  WPI_TalonSRX liftMotor = new WPI_TalonSRX(14);
  WPI_TalonSRX wristMotor = new WPI_TalonSRX(0);

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
  
  Compressor compressor = new Compressor(0);
  DoubleSolenoid ejectorSolenoid = new DoubleSolenoid(1, 3);
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

    SmartDashboard.putNumber("alphavalue", alpha);
    center = operatorJoy.getRawButton(5);
    slowdown = driveJoy.getRawButton(1);
    lineup = driveJoy.getRawButton(2);
    gyromove = driveJoy.getRawButton(3);

    ZRotation = ahrs.getAngle();

    SmartDashboard.putBoolean("slowmode", slowdown);
    SmartDashboard.putBoolean("lineup", lineup);

    SmartDashboard.putNumber("Yaw", ZRotation);
    SmartDashboard.putBoolean("is connected", ahrs.isConnected());
    SmartDashboard.putNumber("XDisp", ahrs.getDisplacementX());
    SmartDashboard.putNumber("YDisp", ahrs.getDisplacementY());

    USSLout = (int) (leftUltrasonic.getAverageVoltage() * 147);
    USSRout = (int) (LeftUltrasonic.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());


    /* Drivetrain Ramping */
    talonFR.configOpenloopRamp(0.20, 20);
    talonFL.configOpenloopRamp(0.20, 20);
    talonBL.configOpenloopRamp(0.20, 20);
    talonBR.configOpenloopRamp(0.20, 20);

    /* Deadzone Logic */
    //Potentially replace with setDeadBand()
    //mecdrive.setDeadband(0.2);
    if (operatorJoy.getRawAxis(0) > deadzone || operatorJoy.getRawAxis(0) < -deadzone)
    {
      xaxis = operatorJoy.getRawAxis(0);
    }
    else
    {
      xaxis = 0;
    }
    if (operatorJoy.getRawAxis(1) > deadzone || operatorJoy.getRawAxis(1) < -deadzone)
    {
      yaxis = operatorJoy.getRawAxis(1);
    }
    else
    {
      yaxis = 0;
    }
    if (driveJoy.getRawAxis(4) > deadzone || driveJoy.getRawAxis(4) < -deadzone)
    {
      rotation = driveJoy.getRawAxis(4);
    }
    else
    {
      rotation = 0;
    }

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
    if(operatorJoy.getRawButton(1))
    {
    ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else if(operatorJoy.getRawButton(2))
    {
    ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    if(operatorJoy.getRawButton(3))
    {
    Solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    else if(operatorJoy.getRawButton(4))
    {
    Solenoid2.set(DoubleSolenoid.Value.kReverse);
    }

     //drive state switcher
     if (driveState.equals("center"))
     {
       if (!center || centerState.equals("stop"))
       {
         tempXAxis = 0;
         mecdrive.driveCartesian(0,0,0);
         driveState = "normal";
         centerState = "start";
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
      if (center && driveJoy.getRawButtonPressed(3))//X
      {
        driveState = "center";
        centerDirection = "left";
      }
      else if (center && driveJoy.getRawButtonPressed(2))//B
      {
        driveState = "center";
        centerDirection = "right";
      }
      else if (lineup && driveJoy.getRawButtonPressed(4))//Y
      {
        driveState = "lineup";
      }
      /*
      placeholders
      */
      else if (driveJoy.getRawButtonPressed(1))//A, placeholder for potential touch screen control
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

    //Operator states
    if (operatorJoy.getRawAxis(0) > 0.2 || operatorJoy.getRawAxis(1) > 0.2 || operatorJoy.getRawAxis(2) > 0.2)
    {
      liftMotorSafe(operatorJoy.getRawAxis(0));
      rotateWristSafe(operatorJoy.getRawAxis(2));
    }
    else
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    if (wristLimitSwitch == false | wristMotor.getSelectedSensorPosition() == linearEncoderConversion(0))
    {
      wristMotor.set(ControlMode.PercentOutput, operatorJoy.getRawAxis(1));
    }
    else if (hatchFloorPositionButton)
    {
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0);
    }
    else if (hatchingPositionButton)
    {
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0);
    }
    else if (hatchLevel1Button)
    {
      //Level 1 is 1 ft. 7 in
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0);
    }
    else if (hatchLevel2Button)
    {
      //Level 2 is 3 ft. 11 in
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0); 
    }
    else if (hatchLevel3Button)
    {
      //Level 3 is 5 ft. 15 in.
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0); 
    }
    else if (cargoLevel1Button)
    {
      //Level 1 is 2 ft. 3.5 in.
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0); 
    }
    else if (cargoLevel2Button)
    {
      //Level 2 is 4 ft. 7.5 in.
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0); 
    }
    else if (cargoLevel3Button)
    {
      //Level 3 is 6 ft. 11.5 in.
      liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
      wristMotor.set(ControlMode.Position, 0); 
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

    driveXAxis = driveJoy.getRawAxis(0);
    driveYAxis = driveJoy.getRawAxis(1);
    driveRotation = driveJoy.getRawAxis(4);
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

    driveXAxis = driveJoy.getRawAxis(0);
    driveYAxis = driveJoy.getRawAxis(1);
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
          centerState = "stop";
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

  public double linearEncoderConversion(double targetInches)
  {
    //The sprocked circumference is either 10 or 12.89
    //The equation is sprocketCircumference/encoderTicksPerRevolution/gearRatio
    return 10/1024/3;
  }

  public void liftMotorSafe(double speed)
  {
    //The limits will need to be changed
    if (liftMotor.getSelectedSensorPosition() == 0 && speed > 0) //Upper limit
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    else if(liftLimitSwitch == true && speed < 0) //Lower limit
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    else // No limits hit
    {
      liftMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void rotateWristSafe(double speed)
  {
    // Limits that will need to be changed
    if (wristMotor.getSelectedSensorPosition() == 0 && speed > 0) //Upper Limit
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else if(wristLimitSwitch == true && speed < 0) //Lower Limit
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else //No limits hit
    {
      wristMotor.set(ControlMode.PercentOutput, speed);
    }
  }
}