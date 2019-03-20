package frc.robot;

<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.kauailabs.navx.frc.*;

>>>>>>> WristPotentiomter
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.cscore.AxisCamera;
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
  /*double xaxis;
  double yaxis;
  double rotation;
  double deadzone = 0.20;
  boolean liftup;
  boolean liftdown;*/

/* Manual drive variables */
  double driveXAxis;
  double driveYAxis;
  double driveRotation;

/* Ultrasonic Straightening Variables */
  double USSLout = 0;
  double USSRout = 0;
  double lineUS;

  /* Alignment Variables */
  /*double gyroangle;
  boolean lineup;
  boolean gyromove;
  boolean center = false;
  boolean centerLeft = false;
  boolean centerRight = false;*/
  private double centerStrafeSpeed = 0.7;
  private double centerReverseSpeed = 0.4;
  private double centerReverseSpeed2 = 0.38;
  private int centerIterations = 10;//how many iterations to reverse
  private int centerIterations2 = 10;
  private int centerIterationsCount = 0;//the count of iterations to reverse
  //Color sensor error estimate
  //private int alphaFirst = 0;

  //Climb
  private double maxCorrection = 0.3;

  /* Automated Drive Variables */
  private double tempXAxis = 0;
  private double tempYAxis = 0;
  private double tempRotation = 0;
  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  /* Robot States */
  private String driveState = "normal";
  private String operatorState = "auto";
  private String hatchCargoPosition = "startingPosition";
  private String centerState = "start";
  private String centerDirection = "left";
  private String lineupState = "start";
  private String climbState = "start";
  private String climbLevel = "3";
  //private int gyroDirection = 4;//facing front
  boolean fieldAssistedDrive = true;

  /* Operator Buttons */
  /*boolean hatchLevel1Button;
  boolean hatchLevel2Button;
  boolean hatchLevel3Button;
  boolean cargoLevel1Button;
  boolean cargoLevel2Button;
  boolean cargoLevel3Button;
  boolean disableSafetiesButton;*/

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
  boolean clear = true;

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
  
  /*boolean operatorJoybuttonA = false;
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
  double operatorJoyAxisRighttStickY = 0;*/

  boolean manualON = true;

  boolean operatorShift = false;
  boolean operatorHatchLevel_1 = false;
  boolean operatorHatchLevel_2 = false;
  boolean operatorHatchLevel_3 = false;
  boolean operatorCargoLevel_1 = false;
  boolean operatorCargoLevel_2 = false;
  boolean operatorCargoLevel_3 = false;

  boolean climb1stLevel = false;
  boolean climb3rdLevel = false;

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

  double wristTolerance = 0.1;
  
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

  int i = 0;

  private final double integrationTime = 10;
  private final short threshold = 25;
  private ByteBuffer buffer = ByteBuffer.allocate(10);
  private short alpha = 0; 

  I2C colorSensor = new I2C(I2C.Port.kOnboard, 0x39);

// Everything Else
  //Instantiate the navx-mxp
  double heading = 0;

  /* Check all IDs */
  /* Wrist Potentiometer for ID 2. */
  AnalogInput wristPot = new AnalogInput(2);

  AnalogInput leftUltrasonic = new AnalogInput(0);
  AnalogInput rightUltrasonic = new AnalogInput(1);

  double liftaxis;
  DigitalInput firstUpperLiftSwitch = new DigitalInput(6);
  DigitalInput secondUpperLftSwitch= new DigitalInput(5);
  DigitalInput bottomLiftSwitch = new DigitalInput(7);
  DigitalInput insideFrameWristSwitch = new DigitalInput(4);
  DigitalInput climbLSwitch = new DigitalInput(3);
  DigitalInput climbRSwitch = new DigitalInput(0);

  WPI_TalonSRX talonFR = new WPI_TalonSRX(0);
  WPI_TalonSRX talonBR = new WPI_TalonSRX(3);

  WPI_TalonSRX talonFL = new WPI_TalonSRX(15);
  WPI_TalonSRX talonBL = new WPI_TalonSRX(13);

  WPI_TalonSRX liftMotor = new WPI_TalonSRX(14);
  WPI_TalonSRX wristMotor = new WPI_TalonSRX(9);

  WPI_TalonSRX climbL = new WPI_TalonSRX(12);
  WPI_TalonSRX climbR = new WPI_TalonSRX(1);

  WPI_TalonSRX WclimbL = new WPI_TalonSRX(8);
  WPI_TalonSRX WclimbR = new WPI_TalonSRX(7);

  WPI_VictorSPX leftCargoIntake = new WPI_VictorSPX(6);
  WPI_VictorSPX rightCargoIntake = new WPI_VictorSPX(10);

  MecanumDrive mecdrive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);
  
  Compressor compressor = new Compressor(0);
  Solenoid ejectorSolenoid = new Solenoid(6);
  DoubleSolenoid Solenoid2 = new DoubleSolenoid(4, 5);
// ----------------------------------------------------------------------------------------
  @Override
  public void robotInit()
  {
      /* Limit Switches */
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    /* Initialize and Configure Cameras */
    new Thread(new Runnable() {
      @Override
      public void run()
      {
        UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        camera1.setResolution(128, 96);
        camera2.setResolution(128, 96);
        camera1.setFPS(30);
        camera2.setFPS(30);
      }
    }).start();

    // Color Sensor Startup Code
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    colorSensor.write(CMD | 0x00, PON | AEN | PEN);
    colorSensor.write(CMD | 0x01, (int) (256-integrationTime/2.38));
    colorSensor.write(CMD | 0x0E, 0b1111);

    // Lift PID Configuration
    liftMotor.setSensorPhase(true);
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMotor.config_kP(0, 0.01, 30);
    liftMotor.config_kI(0, 0.25, 30);
    liftMotor.config_kD(0, 0.015, 30);
    liftMotor.config_kF(0, 0.1, 30);
    liftMotor.configClosedLoopPeriod(0, 1, 30);
    liftMotor.configClearPositionOnQuadIdx(clear, 20);

    // Climb Left PID Configuration
    climbL.setSensorPhase(true);
    climbL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    climbL.config_kP(0, 0.01, 30);
    climbL.config_kI(0, 0.25, 30);
    climbL.config_kD(0, 0.015, 30);
    climbL.config_kF(0, 0.1, 30);
    climbL.configClosedLoopPeriod(0, 1, 30);
    climbL.configClearPositionOnQuadIdx(false, 20);

    // Climb Right PID Configuration
    climbR.setSensorPhase(true);
    climbR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    climbR.config_kP(0, 0.01, 30);
    climbR.config_kI(0, 0.25, 30);
    climbR.config_kD(0, 0.015, 30);
    climbR.config_kF(0, 0.1, 30);
    climbR.configClosedLoopPeriod(0, 1, 30);
    climbR.configClearPositionOnQuadIdx(false, 20);
  }
// ----------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic()
  {
    liftaxis = operatorJoy.getRawAxis(1);
  }  // ----------------------------------------------------------------------------------------
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
    // SmartDashboard Printing and Sensor Configuration
    SmartDashboard.putNumber("alphavalue", alpha);

    USSLout = (int) (leftUltrasonic.getAverageVoltage() * 147);
    USSRout = (int) (rightUltrasonic.getAverageVoltage() * 147);

    SmartDashboard.putData("wrist switch", insideFrameWristSwitch);
    SmartDashboard.putData("First Upper Lift Switch", firstUpperLiftSwitch);
    SmartDashboard.putData("Second Upper Lift Switch", secondUpperLftSwitch);
    SmartDashboard.putData("Bottom Lift Switch", bottomLiftSwitch);
    SmartDashboard.putData("CimbL LS", climbLSwitch);
    SmartDashboard.putData("ClimbR LS", climbRSwitch);

    SmartDashboard.putNumber("Ultrasonic L", USSLout);
    SmartDashboard.putNumber("Ultrasonic R", USSRout);
    SmartDashboard.putNumber("Lift Encoder", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Encoder velocity", liftMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Wrist Potentiometer", wristPot.getAverageVoltage());

    // SmartDashboard Printing
    SmartDashboard.putNumber("alphavalue", alpha);

    SmartDashboard.putBoolean("slowmode", slowdown);
    // SmartDashboard.putBoolean("lineup", lineup);

    SmartDashboard.putNumber("Lift Encoder:", liftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Stilt Encoder", climbL.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Stilt Encoder", climbR.getSelectedSensorPosition());
    
    alpha = readAlphaIntensity();

    // Drive Buttons and Axes
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

    climb1stLevel = driveJoybuttonA;
    climb3rdLevel = driveJoybuttonB;

    // Operator Buttons and Axes
    /*operatorJoybuttonA = operatorJoy.getRawButton(1);
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
    operatorJoyAxisRighttStickY = operatorJoy.getRawAxis(5); */

    operator_ZAxis = operatorJoy.getRawAxis(2);
    operatorShift = operatorJoy.getRawButton(1);

    // Fire Button Logic
    if (operator_ZAxis < 0.85 && operator_ZAxis  > -0.85)
    {
      pneumaticsFire = operatorJoy.getRawButton(2);
      cargoIntake = false;
      cargoEject = false;
    }
    else if (operator_ZAxis >= 0.85 )
    {
      cargoEject = operatorJoy.getRawButton(2);
      cargoIntake = false;
      pneumaticsFire = false;
    }
    else if (operator_ZAxis <= -0.85)
    {
      cargoIntake = operatorJoy.getRawButton(2);
      cargoEject = false;
      pneumaticsFire = false;
    }

    // Operator Shift Button Logic
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

  // Cargo Eject Logic
    if (cargoEject)
    {
      leftCargoIntake.set(-0.45);
      rightCargoIntake.set(0.45);
    }
    else if (cargoIntake)
    {
      leftCargoIntake.set(0.45);
      rightCargoIntake.set(-0.45);
    }
    else
    {
      leftCargoIntake.set(-0.15);
      rightCargoIntake.set(0.15);
    }

  // Pneumatics Fire Logic
    if(pneumaticsFire)
    {
      ejectorSolenoid.set(true);
     // Solenoid2.set(DoubleSolenoid.Value.kForward);
    }
    else
    {
      ejectorSolenoid.set(false);
     // Solenoid2.set(DoubleSolenoid.Value.kReverse);
    }

  // Drive State Logic
    if (driveState.equals("center"))
    {
      if ((centerDirection.equals("left") && !lineFollower_Left) || (centerDirection.equals("right") && !lineFollower_Right) || centerState.equals("stop"))
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
    /* else if (driveState.equals("lineup"))
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
    } */


    else if (driveState.equals("climb"))
    {
      if ((climbLevel.equals("1") && !climb1stLevel) || (climbLevel.equals("3") && !climb3rdLevel) || climbState.equals("stop"))
      {
        tempXAxis = 0;
        tempYAxis = 0;
        tempRotation = 0;
        mecdrive.driveCartesian(0,0,0);
        driveState = "normal";
        climbState = "start";
        operatorState = "auto";
      }
    }
  else
  {
    if (lineFollower_Left && driveJoybuttonLeftBumperPressed) //X
    {
      driveState = "center";
      centerDirection = "left";
    }
    else if (lineFollower_Right && driveJoybuttonRightBumperPressed)
    {
      driveState = "center";
      centerDirection = "right";
    }
   /* else if (driveJoybuttonY) //Y
    {
      driveState = "lineup";
    } */
    else if (climb1stLevel && driveJoybuttonAPressed)
    {
      driveState = "climb";
      climbLevel = "1";
    }
    else if (climb3rdLevel && driveJoybuttonBPressed)
    {
      driveState = "climb";
      climbLevel = "3";
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
     /* case "lineup":
      {
        driveLineup();
        break; 
      } */

      //climb state
      case "climb":
      {
        driveClimb();
        break;
      }//end of climb

    }//end of switch

    //Operator State Switcher
    if (manualON)
    {
      operatorState = "manual";
    }
   /* else if (operatorHatchLevel_1)
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
    /*else if (startingPositionButton)
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
*/
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

      case "stop":
      {
        break;
      }
    }
  }
    // ----------------------------------------------------------------------------------------
  @Override
  public void testPeriodic()
  {

  }
  public short readAlphaIntensity()
  {

    // Light Sensor Configuration
    buffer.clear();
    colorSensor.read(CMD | MULTI_BYTE_BIT | CDATA_REGISTER, 10, buffer);
    short alpha_value = buffer.getShort(0);
    if(alpha_value < 0) { alpha_value += 0b10000000000000000; }
    return alpha_value;
  }

  // Drive Code may be Redundant
  public void driveNormal()
  {

   // mecdrive.setDeadband(0.1);
    if (slowdown)
    {
      slowmodifer = 0.5;
    }
    else
    {
      slowmodifer = 1.0;
    }
    driveXAxis = driveJoyAxisLeftStickX;
    driveYAxis = driveJoyAxisLeftStickY;
    driveRotation = driveJoyAxisRightStickX;
  
      mecdrive.driveCartesian(driveXAxis , -driveYAxis , driveRotation);
  
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
          //alphaFirst = alpha;
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

      //lineup using ultrasonic sensors
     /* case "lineup":
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
        if (USSLout <= 12)
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
      } */
    }

    mecdrive.driveCartesian(tempXAxis,tempYAxis,tempRotation);
  } 

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

  public void trackedStilts(double baseSpeed)
  {
    double leftStilt = climbL.getSelectedSensorPosition();
    double rightStilt = climbR.getSelectedSensorPosition();
    double stiltError = Math.abs(leftStilt - rightStilt);

    // --- Change Multiplier
    double stiltThreshold = 10000;
    
    double multiplier = stiltError / (stiltThreshold / maxCorrection);

    if ((stiltError >= stiltThreshold) && (leftStilt <= rightStilt))
    {
      moveStiltsSafeAuto(baseSpeed, baseSpeed - maxCorrection);//Assume positive is up
    }
    else if ((stiltError >= stiltThreshold) && (leftStilt > rightStilt))
    {
      moveStiltsSafeAuto(baseSpeed - maxCorrection, baseSpeed);
    }
    else if ((stiltError < stiltThreshold) && (leftStilt <= rightStilt))
    {
      moveStiltsSafeAuto(baseSpeed, baseSpeed - multiplier);
    }
    else if ((stiltError < stiltThreshold) && (leftStilt > rightStilt))
    {
      moveStiltsSafeAuto(baseSpeed - multiplier, baseSpeed);
    }
  }

  public void trackedClimb_to_Lift(double baseSpeed)
  {
    double lift = liftMotor.getSelectedSensorPosition();
    double leftStilt = climbL.getSelectedSensorPosition();
    double rightStilt = climbR.getSelectedSensorPosition();
    double maximumStiltPosition = Math.max(leftStilt, rightStilt);
    double liftStiltError = Math.abs(lift - Math.max(leftStilt, rightStilt));
    double liftThreshold = 100;
    double multiplier = liftStiltError / (liftThreshold / maxCorrection);


    if ((liftStiltError > liftThreshold) && (lift >= -maximumStiltPosition))
    {
      liftMotorSafe(baseSpeed - maxCorrection);
    }
    else if ((liftStiltError > liftThreshold) && (lift < -maximumStiltPosition))
    {
      liftMotorSafe(baseSpeed + maxCorrection);
    }
    if ((liftStiltError < liftThreshold) && (lift >= -maximumStiltPosition))
    {
      liftMotorSafe(baseSpeed - multiplier);
    }
    else if ((liftStiltError < liftThreshold) && (lift < -maximumStiltPosition))
    {
      liftMotorSafe(baseSpeed + multiplier);
    }
  }

  public void driveClimb()
  {
    switch (climbState)
    {
      //start climb
      case "start":
      {
        if ((liftMotor.getSelectedSensorPosition() < 2050) && (wristMotor.getSelectedSensorPosition() > 1950))
        {
          climbState = "up";
        }

        else
        {
          operatorState = "stop";
          if (climbLevel.equals("1"))
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 1000);//coming down from above
          }
          else if (climbLevel.equals("3"))
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 2000);//coming down from above
          }
        }

        break;
      }

      case "up":
      {
        if (climbLevel.equals("1"))
        {
          if ((liftMotor.getSelectedSensorPosition() < 100) || (climbL.getSelectedSensorPosition() > 2950) || (climbR.getSelectedSensorPosition() > 2950)) 
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 50);
            climbL.set(ControlMode.Position, 3000);
            climbR.set(ControlMode.Position, 3000);
            climbState = "forward1_start";
          }
  
          else
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            trackedStilts(0.8);
            trackedClimb_to_Lift(0.4);
          }
        }
        else if (climbLevel.equals("3"))
        {
          if ((liftMotor.getSelectedSensorPosition() < 100) || (climbL.getSelectedSensorPosition() > 4950) || (climbR.getSelectedSensorPosition() > 4950)) 
         {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 50);
            climbL.set(ControlMode.Position, 5000);
            climbR.set(ControlMode.Position, 5000);
            climbState = "forward1_start";
          }

          else
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            trackedStilts(0.8);
            trackedClimb_to_Lift(0.4);
          }
        }

        break;
      }

      case "forward1_start":
      {
        timer.reset();
        timer.start();
        climbState = "forward1_run";
        break;
      }

      case "forward1_run":
      {
        if (timer.get() > 1)
        {
          climbState = "forward2_start";
        }

        else
        {
          if (climbLevel.equals("1"))
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 50);
            climbL.set(ControlMode.Position, 3000);
            climbR.set(ControlMode.Position, 3000);
            mecdrive.driveCartesian(0, 0.4, 0);
          }
          else if (climbLevel.equals("3"))
          {
            wristMotor.set(ControlMode.Position, 2000);//all the way down
            liftMotor.set(ControlMode.Position, 50);
            climbL.set(ControlMode.Position, 5000);
            climbR.set(ControlMode.Position, 5000);
            mecdrive.driveCartesian(0, 0.4, 0);
          }
          
        }

        break;
      }

      case "forward2_start":
      {
        timer.reset();
        timer.start();
        climbState = "forward2_run";
        break;
      }

      case "forward2_run":
      {
        if (timer.get() > 1)
        {
          mecdrive.driveCartesian(0, 0, 0);
          climbState = "retract_stilts";
        }

        else
        {
          if (climbLevel.equals("1"))
          {
            wristMotor.set(ControlMode.Position, 100);//up
            liftMotor.set(ControlMode.Position, 500);//up a little
            climbL.set(ControlMode.Position, 5000);
            climbR.set(ControlMode.Position, 5000);
            mecdrive.driveCartesian(0, 0.4, 0);
          }
          else if (climbLevel.equals("3"))
          {
            wristMotor.set(ControlMode.Position, 100);//up
            liftMotor.set(ControlMode.Position, 500);//up a little
            climbL.set(ControlMode.Position, 3000);
            climbR.set(ControlMode.Position, 3000);
            mecdrive.driveCartesian(0, 0.4, 0);
          }
        }

        break;
      }

      case "retract_stilts":
      {
        if ((climbL.getSelectedSensorPosition() < 100) || (climbR.getSelectedSensorPosition() < 100))
        {
          climbL.set(ControlMode.Position, 50);
          climbR.set(ControlMode.Position, 50);
          climbState = "forward3_start";
        }

        else
        {
          wristMotor.set(ControlMode.Position, 100);//up
          liftMotor.set(ControlMode.Position, 500);//up a little
          trackedStilts(-0.8);
        }

        break;
      }

      case "forward3_start":
      {
        timer.reset();
        timer.start();
        climbState = "forward3_run";
        break;
      }

      case "forward3_run":
      {
        if (timer.get() > 1.5)
        {
          mecdrive.driveCartesian(0, 0, 0);
          climbState = "stop";
        }

        else
        {
          wristMotor.set(ControlMode.Position, 100);//up
          liftMotor.set(ControlMode.Position, 500);//up a little
          climbL.set(ControlMode.Position, 50);
          climbR.set(ControlMode.Position, 50);
          mecdrive.driveCartesian(0, 0.4, 0);
        }

        break;
      }
    }//end of switch

    mecdrive.driveCartesian(tempXAxis,tempYAxis,tempRotation);
  }//end of driveClimb()

  public double linearEncoderConversion(double targetInches)
  {
    //The sprocked circumference is either 10 or 12.89
    //The equation is sprocketCircumference/encoderTicksPerRevolution/gearRatio
    return targetInches * 10/1024/3;
  }

  public void liftMotorSafe(double speed)
  {
    //The limits will need to be changed
    if ( firstUpperLiftSwitch.get() == true && secondUpperLftSwitch.get() == true && speed > 0) //Upper limit
    {
      liftMotor.set(ControlMode.Velocity, 0);
    }
    else if(bottomLiftSwitch.get() == true && speed < 0) //Lower limit
    {
      liftMotor.set(ControlMode.Velocity, 0);
    }
    else // No limits hit
    {
      liftMotor.set(ControlMode.Velocity, speed);
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
    else if(wristPot.getAverageVoltage() == 0 && speed < 0) //Lower Limit
    {
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else //No limits hit
    {
      wristMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void rotateWristPotentiometer(double targetVolts, double tolerance)
  {
    if (Math.abs(wristPot.getAverageVoltage() - targetVolts) >= tolerance)
    {
      wristMotor.set(0);
    }
    else if (wristPot.getAverageVoltage() < targetVolts)
    {
      wristMotor.set(1);
    }
    else
    {
      wristMotor.set(-1);
    }
  }

  public void moveStiltsSafe(String direction)
  {
    // Use this method in operatorManual()
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

  public void moveStiltsSafeAuto(double leftSpeed, double rightSpeed)
  {
    // Use this method for auto climb
    if (climbLSwitch.get() == true || climbRSwitch.get() == true)
    {
      climbL.set(ControlMode.PercentOutput, 0);
      climbR.set(ControlMode.PercentOutput, 0);
    }
    else
    {
      climbL.set(ControlMode.PercentOutput, leftSpeed);
      climbR.set(ControlMode.PercentOutput, rightSpeed);
    }
  }

  public void operatorManual()
  {
    if (operatorJoy.getRawButton(10) == true)
    {
      // trackedStilts(1);
      climbL.set(ControlMode.PercentOutput, 1.0);
      climbR.set(ControlMode.PercentOutput, 1.0);
    }
    else if (operatorJoy.getRawButton(11) == true)
    {
     // trackedStilts(-1);
      climbL.set(ControlMode.PercentOutput, -1.0);
      climbR.set(ControlMode.PercentOutput, -1.0);
    }
    else
    {
      climbL.set(0);
      climbR.set(0);
    }
    
    if (operatorJoy.getRawButton(8) && operatorJoy.getRawButton(9))
    {
      WclimbL.set(0.7);
      WclimbR.set(0.7);
    }
    else
    {
      WclimbL.set(0);
      WclimbR.set(0);
    }

    if (-liftaxis <= 0.15 & -liftaxis >= -0.15)
    {
      liftMotor.set(ControlMode.PercentOutput, 0.0);
    }
    else
    {
      liftMotor.set(ControlMode.PercentOutput, -liftaxis);
    }
   
    if (operatorJoy.getRawButton(6))
    {
      wristMotor.set(ControlMode.PercentOutput, 0.5);
    }
    else if (operatorJoy.getRawButton(7))
    {
      wristMotor.set(ControlMode.PercentOutput, -0.8);
    }
    else
    {
      wristMotor.set(ControlMode.PercentOutput, 0.0);
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
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "hatchFloorPickupPosition":
      {
        // Position for picking hatches up from the floor
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "cargoPickupPosition":
      {
        // Position for cargo
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      //Elevator states
      case "hatchLevel1":
      {
        //move until level 1 (1 ft. 7 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "hatchLevel2":
      {
        //move until level 2 (3 ft. 11 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "hatchLevel3":
      {
        //move until next level 3 (5 ft. 15 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "cargoLevel1":
      {
        //move until level 1 (2 ft. 3.5 in.)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "cargoLevel2":
      {
        //move until level 2 (4 ft. 7.5 in)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
      case "cargoLevel3":
      {
        //move until next level 3 (6 ft. 11.5 in)
        liftMotor.set(ControlMode.Position, linearEncoderConversion(0));
        rotateWristPotentiometer(0, wristTolerance);
        break;
      }
    }
  }
}