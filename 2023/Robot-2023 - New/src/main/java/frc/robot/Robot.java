// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Needed to support NAVX card

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// Needed to support USB cameras

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

// Needed to support LimeLight 2+, also needed to support USB cameras

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This robot uses the DifferentialDrive class and runs the motors with tank drive steering using an xBoxController.
 */

public class Robot extends TimedRobot implements Constants {

  // Pseudo-constants (all are positive here) - set once in dashboard before autononousInit() (require tuning)
  
  public boolean AUTONOMOUS_CLIMB          = false; // assume climb is on!
  public int     STATION                   = 0;    // assume station 2 (best for climb)
  
  public long    CLIMB_REVERSE_TIME        = 3000; // milliseconds
  public double  CLIMB_REVERSE_SPEED       = 0.30;
  public long    CLIMB_FORWARD_TIME        = 3000; // milliseconds
  public double  CLIMB_FORWARD_SPEED       = 0.11;

  public long    STATION_LEAVE_TIME      = 5000; // milliseconds
  public double  STATION_LEAVE_SPEED     = 0.30;
  
  // Pseudo-constants for soft-limits on motors - these should give the initial positions and soft limits for
  // each of these motors use the 2023 Soft Limits spreadsheet to store and calculate when tuning these settings
  // @link https://sixnationspolytechnic-my.sharepoint.com/:x:/g/personal/peter_gehbauer_snpsteam_com/EdhdW_0hFbtFl-daE_AMDZYBPoD2q5ts8WoUC_uVnwrWTg?e=dBcdXR
  //
  // (require tuning)

  public double  MIN_POSITION_ROTATING_ARM = -40;
  public double  MAX_POSITION_ROTATING_ARM = 40;
  
  public double  MIN_POSITION_ARM_ELBOW    = -24514; 
  public double  MAX_POSITION_ARM_ELBOW    = -2000;
  
  public double  MIN_POSITION_WINCH        = -250000;
  public double  MAX_POSITION_WINCH        = 12000;
  
  public double  MIN_POSITION_GRABBER      = -3.9;
  public double  MAX_POSITION_GRABBER      = 4.9;

  // Drive Train Motors

  private final WPI_TalonFX leftMotor1 = new WPI_TalonFX(Constants.CAN_LEFT_A);
  private final WPI_TalonFX leftMotor2 = new WPI_TalonFX(Constants.CAN_LEFT_B);
  private final WPI_TalonFX rightMotor1 = new WPI_TalonFX(Constants.CAN_RIGHT_A);
  private final WPI_TalonFX rightMotor2 = new WPI_TalonFX(Constants.CAN_RIGHT_B);

  // Drive Train and Differential Drive

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final DifferentialDrive    robotDrive = new DifferentialDrive   (leftMotors, rightMotors);

  // Other Motors
  
  private final CANSparkMax grabberMotor  = new CANSparkMax(CAN_GRABBER, MotorType.kBrushless);
  private final CANSparkMax rotatorMotor  = new CANSparkMax(CAN_ROTATE,  MotorType.kBrushless);
  private final WPI_TalonFX armWinchMotor = new WPI_TalonFX(CAN_EXTEND);
  private final WPI_TalonFX armElbowMotor = new WPI_TalonFX(CAN_VERTICAL);

  // Controllers
  
  private final XboxController xBoxDriver   = new XboxController(TANK_DRIVE_XBOX); // used for tank drive
  private final XboxController xBoxOperator = new XboxController(OPERATOR_XBOX);   // used for operating the arm

  // NavX card

  private AHRS    navX;
  private boolean navXError = false;

  // USB cameras

  private UsbCamera         camera1;
  private UsbCamera         camera2;
  private NetworkTableEntry cameraSelection;
 
 // the following are used to support autonomous

  private boolean brakeState;
  private int     autonomousState;
  private long    startClimbTime;

  //autonomous states
  public int currentState = 0;
  public boolean safety = true;
  public boolean retracting = true;
 
  // Support for working with soft limits for devices

  private double rotatingArmPosition;
  private double armElbowPosition;
  private double winchPosition;
  private double grabberPosition;
  
  // Support for autonomous
  
  long startTime;
  long leaveTime;

  private static double rampBand = Constants.DEFAULT_RAMP_BAND; //EW 21 sep 2022
  private static double previousMotorSpeed = 0.0;

//================================================================ ROBOINIT ==========================================================================

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */

  @Override
  public void robotInit() {

     // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    leftMotor1.setInverted(true);    leftMotor2.setInverted(true);
    rightMotor1.setInverted(false);     rightMotor2.setInverted(false);

    armElbowMotor.setInverted(true);

    // Set brakes for all motors (off for drive base, true for the others)

    setBrakes(false);                    // drive base (4 x Falcon 500)
    armElbowMotor.setNeutralMode(NeutralMode.Brake); // Falcon 500
    armWinchMotor.setNeutralMode(NeutralMode.Brake); // Falcon 500
    rotatorMotor .setIdleMode   (IdleMode.kBrake);   // NEO 550 w. SparkMAX
    grabberMotor .setIdleMode   (IdleMode.kBrake);   // NEO 550 w. SparkMAX

    // set current limits

    setFalconCurrentLimit(true, leftMotor1,    35.0, 0.5); // Falcon 500
    setFalconCurrentLimit(true, leftMotor2,    35.0, 0.5); // Falcon 500
    setFalconCurrentLimit(true, rightMotor1,    35.0, 0.5); // Falcon 500
    setFalconCurrentLimit(true, rightMotor2,    35.0, 0.5); // Falcon 500
    setFalconCurrentLimit(true, armElbowMotor, 35.0, 0.5); // Falcon 500
    setFalconCurrentLimit(true, armWinchMotor, 35.0, 0.5); // Falcon 500
    rotatorMotor.setSmartCurrentLimit(25);                                                     // SparkMAX
    grabberMotor.setSmartCurrentLimit(25);                                                     // SparkMAX

    

 // set up the NAVX card

    try {

      // navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
      // http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.

      navX = new AHRS(SPI.Port.kMXP);
      navX.enableLogging(true);

    } // end try
    
    catch (RuntimeException ex) {
      navXError = true;
    } // end catch

    // set up USB cameras

    camera1         = CameraServer.startAutomaticCapture(0);
    camera2         = CameraServer.startAutomaticCapture(1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

    // Initial positions of motors (except drive train)

    armElbowPosition    = armElbowMotor.getSelectedSensorPosition();
    winchPosition       = armWinchMotor.getSelectedSensorPosition();
    rotatingArmPosition = rotatorMotor. getEncoder().getPosition();
    grabberPosition     = grabberMotor. getEncoder().getPosition();

    // Put tunable parameters onto the SmartDashboard. The user may update
    // these and the new values are applied at the start of autonomous.

    SmartDashboard.putBoolean("climb",                DEFAULT_CLIMB);
    SmartDashboard.putNumber ("station",              DEFAULT_STATION);

    SmartDashboard.putNumber ("reverse climb time",   CLIMB_REVERSE_TIME); 
    SmartDashboard.putNumber ("reverse climb speed",  CLIMB_REVERSE_SPEED);    
    SmartDashboard.putNumber ("forward climb time",   CLIMB_FORWARD_TIME);
    SmartDashboard.putNumber ("forward climb speed",  CLIMB_FORWARD_SPEED);

    SmartDashboard.putNumber("station leave time",  STATION_LEAVE_TIME);
    SmartDashboard.putNumber("station leave speed", STATION_LEAVE_SPEED);

    SmartDashboard.putBoolean("soft limits", safety);
    SmartDashboard.putBoolean("retracting arm", retracting);
  
    updateSmartDashboard();

  } // end robotInit()

//============================================================= AUTONOMOUS ============================================================================

  @Override
  public void autonomousInit() {

    // update these pseudo-constants on the SmartDashboard before autonomousInit() starts, then use those values for the duration

    AUTONOMOUS_CLIMB      =        SmartDashboard.getBoolean("climb",                true);
    STATION               = (int)  SmartDashboard.getNumber ("station",              0);
    
    CLIMB_REVERSE_TIME    = (long) SmartDashboard.getNumber ("reverse climb time",   0);
    CLIMB_REVERSE_SPEED   =        SmartDashboard.getNumber ("reverse climb speed",  0.0);
    CLIMB_FORWARD_TIME    = (long) SmartDashboard.getNumber ("forward climb time",   0); 
    CLIMB_FORWARD_SPEED   =        SmartDashboard.getNumber ("forward climb speed",  0.0);

    STATION_LEAVE_TIME  = (long) SmartDashboard.getNumber ("station leave time", 0);
    STATION_LEAVE_SPEED =        SmartDashboard.getNumber ("station leave speed",0.0);

    retracting = SmartDashboard.getBoolean("retracting arm", true);

    if (STATION < 0 || STATION > 5) STATION = 0; // default to station 0 if no proper value assigned

    // The following should give the initial positions and soft limits for each of these motors.
    // Use the 2023 Soft Limits spreadsheet to store and calculate when tuning these settings.
    // @link https://sixnationspolytechnic-my.sharepoint.com/:x:/g/personal/peter_gehbauer_snpsteam_com/EdhdW_0hFbtFl-daE_AMDZYBPoD2q5ts8WoUC_uVnwrWTg?e=dBcdXR

    //soft limits should already be set based off of a static start

    //MIN_POSITION_ARM_ELBOW    = armElbowPosition - 0.0;       // replace the - value with delta Left from spreadsheet
    //MAX_POSITION_ARM_ELBOW    = armElbowPosition + 0.0;       // replace the + value with delta Right from spreadsheet
    
    //MIN_POSITION_WINCH        = winchPosition - 0.0;          // replace the - value with delta Left from spreadsheet
    //MAX_POSITION_WINCH        = winchPosition + 0.0;          // replace the + value with delta Right from spreadsheet
    
    //MIN_POSITION_ROTATING_ARM = rotatingArmPosition - 0.0;    // replace the - value with delta Left from spreadsheet
    //MAX_POSITION_ROTATING_ARM = rotatingArmPosition + 0.0;    // replace the + value with delta Right from spreadsheet

    //MIN_POSITION_GRABBER      = grabberPosition - 0.0;        // replace the - value with delta Left from spreadsheet
    //MAX_POSITION_GRABBER      = grabberPosition + 0.0;        // replace the + value with delta Right from spreadsheet

    // set the starting conditions for autonomous

    autonomousState = START;
    startTime       = getTime();

    if(STATION == 0){
      autonomousState = NOTHING;
    }

    updateSmartDashboard();

  } // end autonomousInit()



  /** This function is called periodically during autonomous. */

  @Override
  public void autonomousPeriodic() {

    SmartDashboard.putString("autonomous state", MNEMONIC_AUTONOMOUS_STATES[autonomousState]);
    
    switch (autonomousState) {
 
      case START:

        if(STATION == 5){
          autonomousState = LEAVE;
        } else{
          autonomousState = CUBE;
        }
        
        break;

      // when scoring cones or cubes, consider the scoring during autonomous:
      //    * 3 points when scored on bottom row
      //    * 4 points when scored on middle row
      //    * 6 points when scored on top row
      //
      // We might want to choose scoring a cube in the top row to maximize autonomous points.

      case CUBE:

        // position arm to desired height

        

        if (currentState == 0 && armElbowMotor.getSelectedSensorPosition() > RAISE_ARM_FOR_SCORING) { // think about soft limits too
          armElbowMotor.set(ARM_ELBOW_SPEED);
        } else if(currentState == 0){
          currentState++;
        }// end if

        // extend the arm to desired length

        else if (currentState == 1 && armWinchMotor.getSelectedSensorPosition() > EXTEND_ARM_FOR_SCORING) { // think of soft limits too
          armWinchMotor.set(EXTEND_SPEED);
        } else if(currentState == 1){
          currentState++;
        }// end else if

        // open the grabber to release the cube

        else if (currentState == 2 && grabberMotor.getEncoder().getPosition() > OPEN_GRABBER_FOR_SCORING) { // think of soft limits too
          grabberMotor.set(GRABBER_SPEED);
          // release the cone or cube
        } else if(currentState == 2){
          currentState++;
        }// end else

        // close grabber after scoring

        else if (retracting && currentState == 3 && grabberMotor.getEncoder().getPosition() < CLOSE_GRABBER_AFTER_SCORING) { // think of soft limits too
          grabberMotor.set(-GRABBER_SPEED);
          // release the cone or cube
        } else if(retracting && currentState == 3){
          currentState++;
        }// end else

        // retract arm after scoring

        else if (currentState == 4 && armWinchMotor.getSelectedSensorPosition() < RETRACT_ARM_AFTER_SCORING) { // think of soft limits too
          armWinchMotor.set(RETRACT_SPEED); // <== set the speed for extending the arm
        } else if(currentState == 4){
          currentState++;
        }// end else if

        // lower arm after scoring

        else if (currentState == 5 && armElbowMotor.getSelectedSensorPosition() < LOWER_ARM_AFTER_SCORING) { // think about soft limits too
          armElbowMotor.set(-ARM_ELBOW_SPEED); // <== set the speed for the arm
        } else if(currentState == 5){
          currentState++;
        }// end if

        // move on to either climbing or leaving the community at the end of autonomous

        else {
          if(STATION != 4){
           if (AUTONOMOUS_CLIMB) autonomousState = START_CLIMB; else autonomousState = LEAVE;
          }
        } // end if

        break;

      case CONE:

        autonomousState = CUBE; // deprecated, defaults to CUBE
        break;

      case START_CLIMB:

        // set the timer for timed drive-forward action

        startClimbTime  = getTime();
        autonomousState = CLIMB;
        break;

      case CLIMB:
 
        // The climb does the following: 
        //    * back-up across the platform
        //    * leave the community
        //    * drove forward onto the platform
        //    * level
        //
        // This allows a score of:
        //    * 3 points for leaving the community
        //    * 8 points for docking/levelling the platform.

        if ((getTime() - startClimbTime) < CLIMB_FORWARD_TIME) {
          driveAction(-CLIMB_REVERSE_SPEED); // continue backwards to beyond the platform
        } // end if

        else if ((getTime() - startTime) < (CLIMB_FORWARD_TIME + CLIMB_REVERSE_TIME)) {
          driveAction(CLIMB_FORWARD_SPEED); // now drive forward to climb the platform
        } // end else if

        else {

          setBrakes(true); // ser motor brakes then start balancing process
          autonomousState = BALANCE;

        } // end else

        break;

      case BALANCE:

        // balance the platform (use these to help: navX.getYaw();  navX.getPitch();  navX.getRoll(); )

        if      (navX.getRoll() < -LEVEL_TOLERANCE) driveAction(-CLIMB_FORWARD_SPEED); // front end is pointing downward, reverse a little pitch and roll are reversed
        else if (navX.getRoll() > LEVEL_TOLERANCE)  driveAction( CLIMB_FORWARD_SPEED); //front end is pointing upward, forward a little ^
        else                                         autonomousState = DONE;            // balanced, all done

        break;

      case LEAVE:

         leaveTime = getTime(); // now leave the station that you are at
         if((getTime() - leaveTime) <= STATION_LEAVE_TIME) driveAction(STATION_LEAVE_SPEED); // drive forward for a time then done
        else                                                autonomousState = DONE;             // done
         break;

      case DONE:

        driveAction(0.0); setBrakes(true); // stop, apply brakes and wait until autonomous ends

        if ((getTime() - startTime) > AUTONOMOUS_TIME) autonomousState = EXIT; // hang around until autonomous ends

        break;

      case EXIT:

        // all done with autonomous, remove the brakes on the drive train - teleOperation starts
        setBrakes(false);
        break;

      default:

        autonomousState = DONE;

    } // end switch

    SmartDashboard.putString("autonomous state", MNEMONIC_AUTONOMOUS_STATES[autonomousState]);

    updateSmartDashboard();

  } // end autonomousPeriodic()

//================================================================ TELEOP ============================================================================

  @Override
  public void teleopInit() {

    updateSmartDashboard();

  } // end teleopInit()



  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() {

    safety = SmartDashboard.getBoolean("soft limits", true);//check if safety is on

    // drive the robot (driver)

    //driveAction(xBoxDriver.getLeftY() * .55,xBoxDriver.getRightY() * .55);
    arcadeDrive(xBoxDriver.getLeftY() * .55, xBoxDriver.getRightX() * .55);
    brakeAction();

    // operate the robot (operator)

    //keeps the elbow in place

    motorAction(rotatorMotor,  xBoxOperator.getLeftX() * .2,  MIN_POSITION_ROTATING_ARM, MAX_POSITION_ROTATING_ARM);
    motorAction(armElbowMotor, xBoxOperator.getLeftY() * .25,  MIN_POSITION_ARM_ELBOW,    MAX_POSITION_ARM_ELBOW);
    motorAction(grabberMotor,  xBoxOperator.getRightTriggerAxis() * .1, -xBoxOperator.getLeftTriggerAxis() * .1, MIN_POSITION_GRABBER,      MAX_POSITION_GRABBER);
    motorAction(armWinchMotor, xBoxOperator.getRightY() * .5, MIN_POSITION_WINCH,        MAX_POSITION_WINCH);

    // control the cameras (driver or operator)

    //chooseCamera(); unneccesary

    updateSmartDashboard();

  } // end teleopPeriodic()


  // drive the robot applying ramping as required

  private void driveAction(double leftSpeed, double rightSpeed) {
    robotDrive.tankDrive(leftRamping(deadband(leftSpeed)), rightRamping(deadband(rightSpeed)));
    //robotDrive.tankDrive(deadband(leftSpeed), deadband(rightSpeed));
  } // end driveAction()

  private void arcadeDrive(double speed, double turn){
    robotDrive.arcadeDrive(applyRampBand(deadband(speed)), deadband(turn));
  }

  private void driveAction(double speed) {
    robotDrive.tankDrive(speed, speed);
  } // end driveAction()




  // set or release the brakes on the drivetrain

  private void brakeAction() {
    if (xBoxDriver.getBButton()) setBrakes(true);
    if (xBoxDriver.getXButton()) setBrakes(false);
  } //



  // apply deadband on speed and soft limits to motor position (SparkMAX)

  private void motorAction(CANSparkMax motor, double speed, double minPosition, double maxPosition) {

    double position = motor.getEncoder().getPosition();
    if(safety){
      if      (position <= minPosition) speed = Math.max(0.0, speed); // must be positive
      else if (position >= maxPosition) speed = Math.min(0.0, speed); // must be negative
    }
    motor.set(deadband(speed));

  } // end motorAction()

  private void motorAction(CANSparkMax motor, double speed, double speed2, double minPosition, double maxPosition) {

    if(Math.abs(speed2) > Math.abs(speed)){
      speed = speed2;
    }
    double position = motor.getEncoder().getPosition();
    if(safety){
      if      (position <= minPosition) speed = Math.max(0.0, speed); // must be positive
      else if (position >= maxPosition) speed = Math.min(0.0, speed); // must be negative
    }
    motor.set(deadband(speed));

  } // end motorAction()

  // apply deadband on speed and soft limits to motor position (Falcon 500)

  private void motorAction(WPI_TalonFX motor, double speed, double minPosition, double maxPosition) {

    double position = motor.getSelectedSensorPosition();
    if (safety){
      if      (position <= minPosition) speed = Math.max(0.0, speed); // must be positive
      else if (position >= maxPosition) speed = Math.min(0.0, speed); // must be negative
    }
    motor.set(deadband(speed));

  } // end motorAction()



  // use left or right trigger on either the driver or the operator xbox to select front view camera feed, update SmartDashboard



  private void setBrakes(boolean applyBrakes) {
   
    if ((! brakeState) && applyBrakes) {
      leftMotor1.setNeutralMode(NeutralMode.Brake);    leftMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor1.setNeutralMode(NeutralMode.Brake);    rightMotor2.setNeutralMode(NeutralMode.Brake);
      brakeState = true;
    } // end if

    else if (brakeState) {
      leftMotor1.setNeutralMode(NeutralMode.Coast);    leftMotor2.setNeutralMode(NeutralMode.Coast);
      rightMotor1.setNeutralMode(NeutralMode.Coast);    rightMotor2.setNeutralMode(NeutralMode.Coast);
      brakeState = false;  
    } // end else if

  } // end setBrakes



  private double deadband (double speed) {
    if ((-Constants.DEADBAND < speed) && (speed < Constants.DEADBAND)) return 0.0;
    return speed;
  } // end deadband()



  private void setFalconCurrentLimit(boolean enable, WPI_TalonFX motor, double currentLimit, double currentLimitDuration) {
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enable, currentLimit, currentLimit, currentLimitDuration));
  } // end setFalconCurrentLimit()



  // Because slewRateLimiters filters have 'memory', each input stream requires its own filter object. 
  // DO NOT attempt to use the same filter object for multiple input streams.

  private double applyRampBand(double motorSpeed) {
    if (motorSpeed - previousMotorSpeed < -rampBand) {
      motorSpeed = previousMotorSpeed - rampBand;
    } // end if 
    else if (motorSpeed - previousMotorSpeed > rampBand) {
      motorSpeed = previousMotorSpeed + rampBand;
    } // end else if
  
    previousMotorSpeed = motorSpeed; 
    return motorSpeed;
  }

  private SlewRateLimiter leftRateLimiter = new SlewRateLimiter(RAMPING);  // for left side of drivetrain only
  private double leftRamping (double speed) {                              // for left side of drivetrain only
    return leftRateLimiter.calculate(speed);
  } // end leftRamping()

  private SlewRateLimiter rightRateLimiter = new SlewRateLimiter(RAMPING);  // for right side of drivetrain only
  private double rightRamping(double speed) {                               // for right side of drivetrain only
    return rightRateLimiter.calculate(speed);
  } // end rightRamping()

  private SlewRateLimiter forwardRamp = new SlewRateLimiter(RAMPING);  // for right side of drivetrain only
  private double forwardRamping(double speed) {                               // for right side of drivetrain only
    return forwardRamp.calculate(speed);
  }

  private SlewRateLimiter turnRamp = new SlewRateLimiter(RAMPING);  // for right side of drivetrain only
  private double turnRamping(double speed) {                               // for right side of drivetrain only
    return turnRamp.calculate(speed);
  }

  /** returns the time in milliseconds */

  private long getTime() {
    return System.currentTimeMillis();
  } // end getTime();




//==================================================== PERIODIC SMARTDASHBOARD UPDATING ==============================================================

  /** Called periodically to update the SmartDashboard */

  private void updateSmartDashboard() {

    // device positions (these are useful when (manually) tuning the robot's soft limits)

    SmartDashboard.putNumber ("rotating arm", rotatorMotor .getEncoder().getPosition());
    SmartDashboard.putNumber ("elbow",        armElbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber ("winch",        armWinchMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber ("grabber",      grabberMotor .getEncoder().getPosition());
    SmartDashboard.putBoolean("brakes",       brakeState);
    
    // Set the values of these pseudo-constants based on what was entered into the SmartDashboard before autonomousInit()

    SmartDashboard.putNumber("reverse climb time",    CLIMB_REVERSE_TIME);
    SmartDashboard.putNumber("reverse climb speed",   CLIMB_REVERSE_SPEED);
    SmartDashboard.putNumber("forward climb time",    CLIMB_FORWARD_TIME);
    SmartDashboard.putNumber("forward climb speed",   CLIMB_FORWARD_SPEED); 
    
    SmartDashboard.putNumber("station leave time",  STATION_LEAVE_TIME);
    SmartDashboard.putNumber("station leave speed", STATION_LEAVE_SPEED);

    // The following supports the NavX card

    if (navXError) {
      SmartDashboard.putString ("Version", "error");
    } // end if

    else {
      SmartDashboard.putString ("Version", navX.getFirmwareVersion());
      SmartDashboard.putNumber ("Yaw",     navX.getYaw());   // degrees
      SmartDashboard.putNumber ("Pitch",   navX.getPitch()); // degrees
      SmartDashboard.putNumber ("Roll",    navX.getRoll());  // degrees
    } // end else
    
  } // end updateSmartDashboard()



//================================================================ TESTING ============================================================================

  @Override
  public void testInit() {

    System.out.println("Test Run Start");
    System.out.println("--------------/n");

    cameraSelection.setString(camera1.getName());
    System.out.println("0 - front camera selected");

    updateSmartDashboard();

  } // end testInit()



  @Override
  public void testPeriodic() {

    // test camera switching -------------------------------------------------------------------------------------------------------------------------
  
    if ((xBoxDriver.getLeftTriggerAxis() >= 0.2) || (xBoxOperator.getLeftTriggerAxis() >= 0.2)) {
      System.out.println("1 - front camera selected");
      cameraSelection.setString(camera1.getName());
      SmartDashboard.putString("camera", "front");
    } // end if

    // Check the right trigger on either the driver or the operator xbox to select rear view camera feed ---------------------------------------------

    else if ((xBoxDriver.getRightTriggerAxis() >= 0.2) || (xBoxOperator.getRightTriggerAxis() >= 0.2)) {
      System.out.println("2 - rear camera selected");
      cameraSelection.setString(camera2.getName());
      SmartDashboard.putString("camera", "rear");
    } // end else if

    // check the two joysticks for left and right side drivetrain operation --------------------------------------------------------------------------
    {
      double leftSpeed = leftRamping(deadband(xBoxDriver.getLeftY()));
      double rightSpeed = rightRamping(deadband(xBoxDriver.getRightY()));

      if ((leftSpeed > 0.0 || rightSpeed > 0.0)) {
        driveAction(leftSpeed, rightSpeed);
        System.out.println("3 - driving: left " + leftSpeed + ", right " + rightSpeed);
      } // end if
      else {
        driveAction(0.0, 0.0);
      } // end else
    }

    // Check the B button to turn on the drive train brakes ------------------------------------------------------------------------------------------

    if (xBoxDriver.getBButton()) {
      System.out.println("4 - brakes on");
      setBrakes(true);
    } // end if

    // Check the A button to turn off the drive train brakes -----------------------------------------------------------------------------------------

    if (xBoxDriver.getXButton()) {
      System.out.println("5 - brakes off");
      setBrakes(false);
    } // end if

    // test arm elbow rotation (up or down) ----------------------------------------------------------------------------------------------------------

    {
      double speed = deadband(xBoxOperator.getLeftY());
      if (speed > DEADBAND) System.out.println("6a - elbow rotated up " + speed);
      if (speed < DEADBAND) System.out.println("6b - elbow rotated down " + speed);
      armElbowMotor.set(speed);  // can rotate upward or downward
    }
  
    // test arm rotation (left and right) ------------------------------------------------------------------------------------------------------------

    {
      double speed = deadband(xBoxOperator.getLeftX());
      if (speed > DEADBAND) System.out.println("7a - arm rotated right " + speed);
      if (speed < DEADBAND) System.out.println("7b - arm rotated left " + speed);
      rotatorMotor.set(speed); // can rotate left or right
    }

    // test arm extending/retracting -----------------------------------------------------------------------------------------------------------------

    {
      double speed = deadband(xBoxOperator.getRightY());
      if (speed > DEADBAND) System.out.println("8a - arm extending " + speed);
      if (speed < DEADBAND) System.out.println("8b - arm retracting " + speed);
      armWinchMotor.set(speed); // can extend or retract
  
    }

    // test grabber opening and closing --------------------------------------------------------------------------------------------------------------

    {
      double speed = deadband(xBoxOperator.getRightX());
      if (speed > DEADBAND) System.out.println("9a - grabber opening " + speed);
      if (speed < DEADBAND) System.out.println("9b - grabber closing " + speed);
      grabberMotor.set(speed); // can open or close
    }

    // test pitch, roll, yaw -------------------------------------------------------------------------------------------------------------------------

    // move the robot at different angles to see what the NavX card pitch, roll and yaw values are in the SmartDashboard

    updateSmartDashboard();

  } // end testPeriodic()

} // end class Robot
