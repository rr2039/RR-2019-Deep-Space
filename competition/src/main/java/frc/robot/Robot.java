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
import edu.wpi.first.wpilibj.DigitalInput;
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
  double USSLout = 0;
  double USSRout = 0;
  double lineUS;

  /* Alignment Variables */
  double gyroangle;
  boolean lineup;
  boolean gyromove;
  boolean center = false;
  boolean centerLeft = false;
  boolean centerRight = false;
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
  private String operatorState = "auto";
  private String hatchCargoPosition = "startingPosition";
  private String centerState = "start";
  private String centerDirection = "left";
  private String lineupState = "start";
  private int gyroDirection = 4;//facing front
  private boolean fieldAssistedDrive = true;

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

  //Controls
  boolean driveJoybuttonA = false;
  boolean driveJoybuttonAPressed = false;
  boolean driveJoybuttonB = false;
  boolean driveJoybuttonBPressed = false;
  boolean driveJoybuttonX = false;
  boolean driveJoybuttonXPressed = false;
  boolean driveJoybuttonY = false;
  boolean driveJoybuttonYPressed = false;
  boolean lineFollower_Left = false;
  boolean driveJoybuttonLeftBumperPressed = false;
  boolean lineFollower_Right = false;
  boolean driveJoybuttonRightBumperPressed = false;
  boolean driveJoybuttonBack = false;
  boolean driveJoybuttonBackPressed = false;
  boolean driveJoybuttonStart = false;
  boolean driveJoybuttonStartPressed = false;
  double driveJoyAxisLeftStickX = 0;
  double driveJoyAxisLeftStickY = 0;
  double driveJoyAxisLeftTrigger = 0;
  double driveJoyAxisRightTrigger = 0;
  double driveJoyAxisRightStickX = 0;
  double driveJoyAxisRighttStickY = 0;

  boolean operatorJoybuttonA = false;
  boolean operatorJoybuttonAPressed = false;
  boolean operatorJoybuttonB = false;
  boolean operatorJoybuttonBPressed = false;
  boolean operatorJoybuttonX = false;
  boolean operatorJoybuttonXPressed = false;
  boolean operatorJoybuttonY = false;
  boolean operatorJoybuttonYPressed = false;
  boolean operatorJoybuttonLeftBumper = false;
  boolean operatorJoybuttonLeftBumperPressed = false;
  boolean operatorJoybuttonRightBumper = false;
  boolean operatorJoybuttonRightBumperPressed = false;
  boolean operatorJoybuttonBack = false;
  boolean operatorJoybuttonBackPressed = false;
  boolean operatorJoybuttonStart = false;
  boolean operatorJoybuttonStartPressed = false;
  double operatorJoyAxisLeftStickX = 0;
  double operatorJoyAxisLeftStickY = 0;
  double operatorJoyAxisLeftTrigger = 0;
  double operatorJoyAxisRightTrigger = 0;
  double operatorJoyAxisRightStickX = 0;
  double operatorJoyAxisRighttStickY = 0;

  boolean operatorShift = false;
  boolean operatorHatchLevel_1 = false;
  boolean operatorHatchLevel_2 = false;
  boolean operatorHatchLevel_3 = false;
  boolean operatorCargoLevel_1 = false;
  boolean operatorCargoLevel_2 = false;
  boolean operatorCargoLevel_3 = false;

  double operator_ZAxis;
  boolean pneumaticsFire = false;
  boolean cargoEject = false;
  boolean cargoIntake = false;

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
  
  // Hatch and Cargo scoring wrist positions
  final double wristHatchLevel_Position = 0;
  final double wristCargoLevel_Position = 0;


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
  double heading = 0;

  /* Check all IDs */
  AnalogInput leftUltrasonic = new AnalogInput(0);
  AnalogInput rightUltrasonic = new AnalogInput(1);

  DigitalInput firstUpperLiftSwitch = new DigitalInput(0);
  DigitalInput secondUpperLftSwitch= new DigitalInput(0);
  DigitalInput bottomLiftSwitch = new DigitalInput(0);
  DigitalInput insideFrameWristSwitch = new DigitalInput(0);
  DigitalInput climbLSwitch = new DigitalInput(0);
  DigitalInput climbRSwitch = new DigitalInput(0);

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

  WPI_VictorSPX leftCargoIntake = new WPI_VictorSPX(10);
  WPI_VictorSPX rightCargoIntake = new WPI_VictorSPX(11);

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
  
  Compressor compressor = new Compressor(0);
 // Solenoid ejectorSolenoid = new Solenoid(4);
  DoubleSolenoid ejectorSolenoid = new DoubleSolenoid(1, 3);
  DoubleSolenoid Solenoid2 = new DoubleSolenoid(0, 2);
// ----------------------------------------------------------------------------------------
  @Override
  public void robotInit()
  {
      /* Limit Switches */

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

    /* Drivetrain Ramping */
    talonFR.configOpenloopRamp(0.25, 20);
    talonFL.configOpenloopRamp(0.25, 20);
    talonBL.configOpenloopRamp(0.25, 20);
    talonBR.configOpenloopRamp(0.25, 20);

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
        mainCode();
        break;
    }
  }
   // ----------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic()
  {
    mainCode();
  }
  public void mainCode()
  {
    //Smart dashboard stuff
    SmartDashboard.putNumber("alphavalue", alpha);
    USSLout = (int) (leftUltrasonic.getAverageVoltage() * 147);
    USSRout = (int) (rightUltrasonic.getAverageVoltage() * 147);
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());


    /* Drivetrain Ramping */
    talonFR.configOpenloopRamp(0.20, 20);
    talonFL.configOpenloopRamp(0.20, 20);
    talonBL.configOpenloopRamp(0.20, 20);
    talonBR.configOpenloopRamp(0.20, 20);

    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("alphavalue", alpha);

    SmartDashboard.putBoolean("slowmode", slowdown);
    SmartDashboard.putBoolean("lineup", lineup);

    SmartDashboard.putNumber("Yaw", heading);
    SmartDashboard.putBoolean("is connected", ahrs.isConnected());
    SmartDashboard.putNumber("XDisp", ahrs.getDisplacementX());
    SmartDashboard.putNumber("YDisp", ahrs.getDisplacementY());
    
    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);

    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());

    //sensors
    heading = ahrs.getAngle();
    USSLout = (int) (leftUltrasonic.getAverageVoltage() * 147);
    USSRout = (int) (rightUltrasonic.getAverageVoltage() * 147);
    alpha = readAlphaIntensity();

    //These have to be done here; otherwise, some "pressed" methods will reset in robotPeriodic before a new teleop loop.
    driveJoybuttonA = driveJoy.getRawButton(1);
    driveJoybuttonAPressed = driveJoy.getRawButtonPressed(1);
    driveJoybuttonB = driveJoy.getRawButton(2);
    driveJoybuttonBPressed = driveJoy.getRawButtonPressed(2);
    driveJoybuttonX = driveJoy.getRawButton(3);
    driveJoybuttonXPressed = driveJoy.getRawButtonPressed(3);
    driveJoybuttonY = driveJoy.getRawButton(4);
    driveJoybuttonYPressed = driveJoy.getRawButtonPressed(4);
    lineFollower_Left = driveJoy.getRawButton(5);
    driveJoybuttonLeftBumperPressed = driveJoy.getRawButtonPressed(5);
    lineFollower_Right = driveJoy.getRawButton(6);
    driveJoybuttonRightBumperPressed = driveJoy.getRawButtonPressed(6);
    driveJoybuttonBack = driveJoy.getRawButton(7);
    driveJoybuttonBackPressed = driveJoy.getRawButtonPressed(7);
    driveJoybuttonStart = driveJoy.getRawButton(8);
    driveJoybuttonStartPressed = driveJoy.getRawButtonPressed(8);
    driveJoyAxisLeftStickX = driveJoy.getRawAxis(0);
    driveJoyAxisLeftStickY = driveJoy.getRawAxis(1);
    driveJoyAxisLeftTrigger = driveJoy.getRawAxis(2);
    driveJoyAxisRightTrigger = driveJoy.getRawAxis(3);
    driveJoyAxisRightStickX = driveJoy.getRawAxis(4);
    driveJoyAxisRighttStickY = driveJoy.getRawAxis(5);

    operatorJoybuttonA = operatorJoy.getRawButton(1);
    operatorJoybuttonAPressed = operatorJoy.getRawButtonPressed(1);
    operatorJoybuttonB = operatorJoy.getRawButton(2);
    operatorJoybuttonBPressed = operatorJoy.getRawButtonPressed(2);
    operatorJoybuttonX = operatorJoy.getRawButton(3);
    operatorJoybuttonXPressed = operatorJoy.getRawButtonPressed(3);
    operatorJoybuttonY = operatorJoy.getRawButton(4);
    operatorJoybuttonYPressed = operatorJoy.getRawButtonPressed(4);
    operatorJoybuttonLeftBumper = operatorJoy.getRawButton(5);
    operatorJoybuttonLeftBumperPressed = operatorJoy.getRawButtonPressed(5);
    operatorJoybuttonRightBumper = operatorJoy.getRawButton(6);
    operatorJoybuttonRightBumperPressed = operatorJoy.getRawButtonPressed(6);
    operatorJoybuttonBack = operatorJoy.getRawButton(7);
    operatorJoybuttonBackPressed = operatorJoy.getRawButtonPressed(7);
    operatorJoybuttonStart = operatorJoy.getRawButton(8);
    operatorJoybuttonStartPressed = operatorJoy.getRawButtonPressed(8);
    operatorJoyAxisLeftStickX = operatorJoy.getRawAxis(0);
    operatorJoyAxisLeftStickY = operatorJoy.getRawAxis(1);
    operatorJoyAxisLeftTrigger = operatorJoy.getRawAxis(2);
    operatorJoyAxisRightTrigger = operatorJoy.getRawAxis(3);
    operatorJoyAxisRightStickX = operatorJoy.getRawAxis(4);
    operatorJoyAxisRighttStickY = operatorJoy.getRawAxis(5); 

    operator_ZAxis = operatorJoy.getRawAxis(2);
    operatorShift = operatorJoy.getRawButton(1);

    if (operator_ZAxis < 0.85 || operator_ZAxis  > -0.85)
    {
      pneumaticsFire = operatorJoy.getRawButton(2);
      cargoIntake = false;
      pneumaticsFire = false;
    }
    else if (operator_ZAxis > 0.85 )
    {
      cargoEject = operatorJoy.getRawButton(2);
      cargoIntake = false;
      pneumaticsFire = false;
    }
    else if (operator_ZAxis < -0.85)
    {
      cargoIntake = operatorJoy.getRawButton(2);
      cargoEject = false;
      pneumaticsFire = false;
    }
    if (operatorShift)
    {
      operatorCargoLevel_1 = operatorJoy.getRawButton(4);
      operatorCargoLevel_2 = operatorJoy.getRawButton(3);
      operatorCargoLevel_3 = operatorJoy.getRawButton(5);

      operatorHatchLevel_1 = false;
      operatorHatchLevel_2 = false;
      operatorHatchLevel_3 = false;
    }
    else
    {
      operatorHatchLevel_1 = operatorJoy.getRawButton(4);
      operatorHatchLevel_2 = operatorJoy.getRawButton(3);
      operatorHatchLevel_3 = operatorJoy.getRawButton(5);

      operatorCargoLevel_1 = false;
      operatorCargoLevel_2 = false;
      operatorCargoLevel_3 = false;
    }
    if (lineFollower_Left)
    {
      centerDirection = "left";
      driveState = "center";
    }
    else if (lineFollower_Right)
    {
      centerDirection = "right";
      driveState = "center";
    }
    else
    {
      centerDirection = "stop";
      driveState = "normal";
    }

    if (cargoEject)
    {
      leftCargoIntake.set(1.0);
      rightCargoIntake.set(-1.0);
    }
    else if (cargoIntake)
    {
      leftCargoIntake.set(-1.0);
      rightCargoIntake.set(1.0);
    }
    else
    {
      leftCargoIntake.set(0);
      rightCargoIntake.set(0);
    }
    /* Pnumatics Logic */
    if(pneumaticsFire)
    {
      ejectorSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else
    {
      ejectorSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

     //drive state switcher
    if (driveState.equals("center"))
    {
      if ((centerDirection.equals("left") && !driveJoybuttonX) || (centerDirection.equals("right") && ! driveJoybuttonB) || centerState.equals("stop"))
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
      if (!driveJoybuttonY || lineupState.equals("stop"))
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
    if (driveJoybuttonX)//X
    {
      driveState = "center";
      centerDirection = "left";
    }
    else if (operatorJoybuttonB)//B
    {
      driveState = "center";
      centerDirection = "right";
    }
    else if (driveJoybuttonY)//Y
    {
      driveState = "lineup";
    }
    /*
    placeholders
    */
    else if (driveJoybuttonA)//A, placeholder for potential touch screen control
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

    //Operator State Switcher
    if (operatorJoyAxisLeftStickX > 0.2 | operatorJoyAxisLeftStickY > 0.2 | operatorJoyAxisLeftTrigger > 0.2 | operatorJoyAxisRightTrigger > 0.2)
    {
      operatorState = "manual";
    }
    else if (operatorHatchLevel_1)
    {
      operatorState = "auto";
      hatchCargoPosition = "hatchLevel1";
    }
    else if (operatorHatchLevel_2)
    {
      operatorState = "auto";
      hatchCargoPosition = "hatchLevel2";
    }
    else if (operatorHatchLevel_3)
    {
      operatorState = "auto";
      hatchCargoPosition = "hatchLevel3";
    }
    else if (operatorCargoLevel_1)
    {
      operatorState = "auto";
      hatchCargoPosition = "cargoLevel1";
    }
    else if (operatorCargoLevel_2)
    {
      operatorState = "auto";
      hatchCargoPosition = "cargoLevel2";
    }
    else if (operatorCargoLevel_3)
    {
      operatorState = "auto";
      hatchCargoPosition = "cargoLevel3";
    }
    else if (startingPositionButton)
    {
      operatorState = "auto";
      hatchCargoPosition = "startingPosition";
    }
    else if (hatchingPositionButton)
    {
      operatorState = "auto";
      hatchCargoPosition = "hatchPickupPosition";
    }
    else if (hatchFloorPositionButton)
    {
      operatorState = "auto";
      hatchCargoPosition = "hatchFloorPickupPosition";
    }
    else if (cargoPositionButton)
    {
      operatorState = "auto";
      hatchCargoPosition = "cargoPickupPosition";
    }

    switch (operatorState)
    {
      case "manual":
      {
        operatorManual();
        break;
      }
      
      case "auto":
      {
        operatorAuto();
        break;
      }
    }
  }
    // ----------------------------------------------------------------------------------------
  @Override
  public void testPeriodic()
  {

  }
  /* Ultrasonic Lineup Function */
  /*public void USCorrection()
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
  }*/
 
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

    driveXAxis = operatorJoyAxisLeftStickX;
    driveYAxis = operatorJoyAxisLeftStickY;
    driveRotation = operatorJoyAxisRightStickX;
    if (fieldAssistedDrive)
    {
      mecdrive.driveCartesian(driveXAxis * slowmodifer, -driveYAxis * slowmodifer, driveRotation * slowmodifer, ahrs.getAngle());
    }
    else
    {
      mecdrive.driveCartesian(driveXAxis * slowmodifer, -driveYAxis * slowmodifer, driveRotation * slowmodifer);
    }
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

    driveXAxis = operatorJoyAxisLeftStickX;
    driveYAxis = operatorJoyAxisLeftStickY;
    double error = 0;
    double angle = 0;
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
    mecdrive.driveCartesian(driveXAxis * slowmodifer, -driveYAxis * slowmodifer, -error*kp * slowmodifer, error);
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

        else
        {
          driveLineup();
        }

        break;
      }

      //lineup using ultrasonic sensors
      case "drive_in":
      {
        if (USSLout <= 11)
        {
          tempXAxis = 0;
          tempYAxis = 0;
          tempRotation = 0;
          centerState = "stop";
        }

        else
        {
          tempXAxis = 0;
          tempYAxis = 0.4;
          tempRotation = 0;
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

  public double linearEncoderConversion(double targetInches)
  {
    //The sprocked circumference is either 10 or 12.89
    //The equation is sprocketCircumference/encoderTicksPerRevolution/gearRatio
    return 10/1024/3;
  }

  public void liftMotorSafe(double speed)
  {
    //The limits will need to be changed
    if ( firstUpperLiftSwitch.get() == true && secondUpperLftSwitch.get() == true && speed > 0) //Upper limit
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    else if(bottomLiftSwitch.get() == true && speed < 0) //Lower limit
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    else // No limits hit
    {
      liftMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void liftMotor_to_PositionSafe(double encoderPosition)
  {

  }

  public void rotateWristSafe(double speed)
  {
    // Limits that will need to be changed
    if (insideFrameWristSwitch.get() == true && speed > 0) //Inside frame limit
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else if(wristMotor.getSelectedSensorPosition() == 0 && speed < 0) //Lower Limit
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else //No limits hit
    {
      wristMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void moveStiltsSafe(String direction)
  {
    if (climbLSwitch.get() == true || climbRSwitch.get() == true)
    {
      climbL.set(0);
      climbR.set(0);
    }
    else if (direction.equals("up"))
    {
      climbL.set(ControlMode.PercentOutput, 1.0);
      climbR.set(ControlMode.PercentOutput, 1.0);
    }
    else if (direction.equals("down"))
    {
      climbL.set(ControlMode.PercentOutput, -1.0);
      climbR.set(ControlMode.PercentOutput, -1.0);
    }
  }

  public void operatorManual()
  {
    if (operatorJoybuttonA == true)
    {
      moveStiltsSafe("down");
    }
    else if (operatorJoybuttonY == true)
    {
      moveStiltsSafe("up");
    }
    else
    {
      climbL.set(0);
      climbR.set(0);
    }
    
    if (operatorJoybuttonB == true)
    {
      WclimbL.set(operatorJoyAxisLeftTrigger);
      WclimbR.set(operatorJoyAxisRightTrigger);
    }
    else
    {
      WclimbL.set(0);
      WclimbR.set(0);
    }
  
    if (operatorJoybuttonX)
    {
      ejectorSolenoid.set(Value.kForward);
    }
    else
    {
      ejectorSolenoid.set(Value.kOff);
    }

    //This may need to change to a different encoder position
    if (liftMotor.getSelectedSensorPosition() == linearEncoderConversion(0))
    {
      liftMotorSafe(operatorJoyAxisLeftStickX);
    }
    else
    {
      liftMotor.set(ControlMode.PercentOutput, 0);
    }
    if (wristMotor.getSelectedSensorPosition() == wristHatchingFloorPosition)
    {
      rotateWristSafe(operatorJoyAxisLeftStickY);
    }          
    else
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void operatorAuto()
  {
    switch (hatchCargoPosition)
    {
      case "hatchPickupPosition":
      {
        // Position for hatching and picking up from loading station
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristHatchLevel_Position);
        break;
      }
      case "hatchPickupFloorPosition":
      {
        // Position for picking hatches up from the floor
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristHatchingFloorPosition);
        break;
      }
      case "cargoPickupPosition":
      {
        // Position for cargo
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristCargoPickupPosition);
        break;
      }
      //Elevator states
      case "hatchLevel1":
      {
        //move until level 1 (1 ft. 7 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristHatchLevel_Position);
        break;
      }
      case "hatchLevel2":
      {
        //move until level 2 (3 ft. 11 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristHatchLevel_Position);
        break;
      }
      case "hatchLevel3":
      {
        //move until next level 3 (5 ft. 15 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristHatchLevel_Position);
        break;
      }
      case "cargoLevel1":
      {
        //move until level 1 (2 ft. 3.5 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristCargoLevel_Position);
        break;
      }
      case "cargoLevel2":
      {
        //move until level 2 (4 ft. 7.5 in)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristCargoLevel_Position);
        break;
      }
      case "cargoLevel3":
      {
        //move until next level 3 (6 ft. 11.5 in)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        wristMotor.set(ControlMode.Position, wristCargoLevel_Position);
        break;
      }
    }
  }
}