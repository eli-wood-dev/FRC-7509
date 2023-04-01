// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */

public interface Constants {

  // CAN IDs

  public static final int CAN_LEFT_A             = 14;
  public static final int CAN_LEFT_B             = 16;
  public static final int CAN_RIGHT_A            = 15;
  public static final int CAN_RIGHT_B            = 17;
  public static final int CAN_ROTATE             = 20; 
  public static final int CAN_GRABBER            = 21; 
  public static final int CAN_VERTICAL           = 18; 
  public static final int CAN_EXTEND             = 19;
  public static final int POWER_DISTRIBUTION_HUB = 00;

  // set the USB IDs for the driver and operator controls

  public static final int TANK_DRIVE_XBOX = 0;
  public static final int OPERATOR_XBOX   = 1;

  // Support for motor control (may need tuning)

  public static final double DEADBAND = 0.001;
  public static final double RAMPING  = 0.9; // units per second

  public static final double DEFAULT_RAMP_BAND = 0.1;

  public static final double RAMP_RATE = 0.05;
  public static final double TURN_RAMP_RATE = 0.05;

  public static final double MAX_SPEED = 1;
  public static final double MAX_TURN_SPEED = 0.9;

  // Autonomous positions before scoring (may need tuning)

  public final static double RAISE_ARM_FOR_SCORING       = -24514;
  public final static double EXTEND_ARM_FOR_SCORING      = -278000;
  public final static double OPEN_GRABBER_FOR_SCORING    = -3.9;          //ZS

  // Autonomous positions after scoring (may need tuning)

  public final static double CLOSE_GRABBER_AFTER_SCORING = 2;           //ZS changed bc not reaching prev va
  public final static double RETRACT_ARM_AFTER_SCORING   = -1;
  public final static double LOWER_ARM_AFTER_SCORING     = -2050;

  //soft limits

  public double  MIN_POSITION_ROTATING_ARM = -40;
  public double  MAX_POSITION_ROTATING_ARM = 40;
  
  public double  MIN_POSITION_ARM_ELBOW    = -24514; 
  public double  MAX_POSITION_ARM_ELBOW    = -2050;
  
  public double  MIN_POSITION_WINCH        = -278000;
  public double  MAX_POSITION_WINCH        = -1;
  
  public double  MIN_POSITION_GRABBER      = -3.9;
  public double  MAX_POSITION_GRABBER      = 4.9;

  public static final double PASSIVE_ELBOW_SPEED = -0.1;

  // Autonomous motor speeds (all are positive here) (may need tuning)

  public static final double  GRABBER_SPEED              = -0.10;
  public static final double  EXTEND_SPEED               = -0.50;
  public static final double  RETRACT_SPEED              = 0.50;
  public static final double  ARM_ELBOW_SPEED            = -0.25;
  public static final double  ARM_ELBOW_LOWER_SPEED      = 0.1;
  public static final double  LEVEL_TOLERANCE            = 10.0; // degrees, -LEVEL_TOLERANCE .. +LEVEL_TOLERANCE indicates level <== adjust as required

  // Autonomous defaults about whether to climb and which station the robot starts at

  public static final boolean DEFAULT_CLIMB              = false; // default: will not climb
  public static final int     DEFAULT_STATION            = 0;    // default: nothing

  // Support for autonomous (may need tuning)
  
  final long AUTONOMOUS_TIME =  18000; // 18 seconds
  final long ENDGAME_TIME    = 120000; // 120 seconds (2 minutes)

  // Autonomous states (don't change these)
    
  final int START       = 0;  
  final int STATION_1   = 1;
  final int STATION_2   = 2;
  final int STATION_3   = 3;
  final int CONE        = 4; 
  final int CUBE        = 5;
  final int START_CLIMB = 6;
  final int CLIMB       = 7;
  final int BALANCE     = 8;
  final int LEAVE       = 9;
  final int DONE        = 10;
  final int EXIT        = 11;
  final int NOTHING     = 13;

  final String[] MNEMONIC_AUTONOMOUS_STATES = {"start", "station 1", "station 2", "station 3", "cone", "cube",
                                               "start climb", "climb", "balance", "leave", "done", "exit"};

  // Button mappings for robot actions by driver and operator are shown below.
  //
  // These constants were not used. XboxController methods were used for  both controllers - xBoxDriver and xBoxOperator.
  //
  // Control        Driver                      Operator                     Not used Method                Returns
  // -------------- --------------------------- ---------------------------- -------- --------------------- ---------
  // LEFT_TRIGGER   select front camera         select front camera                   getLeftTriggerAxis()   0.0 to 1.0
  // RGHT_TRIGGER   select rear camera          select rear camera                    getRightTriggerAxis()  0.0 to 1.0
  // LEFT_JOY_X                                 rotate arm left or right              getLeftX()            -1.0 to 1.0 
  // LEFT_JOY_Y     drive left side             rotate arm up or down                 getLeftY()            -1.0 to 1.0
  // RGHT_JOY_X                                 extend or retract arm                 getRightX()           -1.0 to 1.0
  // RGHT_JOY_Y     drive right side            open or close grabber                 getRightY()           -1.0 to 1.0
  // BUTTON_A                                                                X
  // BUTTON_B       drive train brakes                                                getBButton()          boolean
  // BUTTON_X       release drive train brakes                                        getXButton()          boolean
  // BUTTON_Y                                                                X
  // BACK                                                                    X
  // START                                                                   X
  // HAT_UP                                                                  X
  // HAT_RIGHT                                                               X
  // HAT_DOWN                                                                X
  // HAT_LEFT                                                                X

  // NOTE: The MODE button may not be a readable button press.  Take care so that an inadvertent MODE button press doesn't adversely 
  //       change how the controller works during teleopPeriodic()!
    
} // end interface Constants
