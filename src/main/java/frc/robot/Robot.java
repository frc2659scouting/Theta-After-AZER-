// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final int PH_CAN_ID = 20;
    private static final int SOLENOID_CHANNEL = 8;
    PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
    Solenoid m_intakeSolenoid = m_pH.makeSolenoid(SOLENOID_CHANNEL);
  private final XboxController m_driveController = new XboxController(0);
    private boolean gamePieceReleaseReset = false;
    private boolean HPDoubleCubeReleaseReset = false;
    private boolean HPDoubleConeReleaseReset = false;
    private final XboxController m_operatorController = new XboxController(1);
    public static final ArmSubsystem m_armSub = new ArmSubsystem();
    //public static PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        
        m_robotContainer = new RobotContainer();
        m_armSub.initialize();
        PortForwarder.add(5800, "photonvision.local", 5800);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_armSub.readHall0();

    if (ArmSubsystem.m_pdh.getCurrent(21) < .4375) {
        //DataLogManager.log("RIO Power : " + ArmSubsystem.m_pdh.getCurrent(21));
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    ArmSubsystem.autonomous = false;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    ArmSubsystem.autonomous = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    RobotContainer.s_Swerve.resetOdometry(new Pose2d());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_intakeSolenoid.set(m_armSub.solenoidPosition());
  
   //m_intakeSolenoid.set(m_armSub.getSolAuto());
  }

   //SmartDashboard.putBoolean("autonomousSolenoid", ArmSubsystem.autonomousSolenoid);
  @Override
  public void teleopInit() {
    ArmSubsystem.autonomous = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (m_driveController.getYButton()) {
      AutoAlign.align();
  } else {
      AutoAlign.disable();
  }
  
   SmartDashboard.putNumber("OperatorPOV", m_operatorController.getPOV());
  
  m_intakeSolenoid.set(m_armSub.solenoidPosition());
  if (m_driveController.getXButtonPressed()) { 
    SwerveSubsystem.resetModulesToAbsolute();
  }

  //This starts the intaking function so that the auto retract will override it, but drivers can take back control if needed 
  if (m_operatorController.getRightBumperPressed() || m_operatorController.getLeftBumperPressed() || m_operatorController.getStartButtonPressed() || m_driveController.getRightBumperPressed() || m_driveController.getLeftBumperPressed() || m_driveController.getStartButtonPressed()) {
      m_armSub.startIntaking();
  } else if (m_operatorController.getRightBumper()) {
      m_armSub.intakeConePointOut();
  } else if (m_operatorController.getLeftBumper()) {
      m_armSub.intakeConePointIn();
  } else if (m_operatorController.getStartButton()) {
      m_armSub.intakeCube();
  }else if (m_operatorController.getStartButtonReleased()) {
      m_armSub.stopIntakingCube();
  } else if (m_operatorController.getXButtonPressed()) {
      m_armSub.goHome();
  } else if (m_operatorController.getAButton()) {
      m_armSub.levelOne();
  } else if (m_operatorController.getBButton()) {
      m_armSub.levelTwo();
  } else if (m_operatorController.getYButton()) {
      m_armSub.levelThree();
  } else if (m_operatorController.getRightTriggerAxis() > 0.7) { //Operator releases gamepiece
      m_armSub.releaseGamePiece();
      gamePieceReleaseReset = true;
  } else if (m_operatorController.getRightTriggerAxis() < 0.7 && gamePieceReleaseReset) {
      m_armSub.unreleaseGamepiece();
      gamePieceReleaseReset = false;
  } else if (m_operatorController.getRawButtonPressed(7)) {
      m_armSub.prescore();
  } else if (m_operatorController.getLeftTriggerAxis() > 0.7) { //Operator releases gamepiece
      m_armSub.traverse();
  } else if (m_driveController.getAButtonPressed()) { //driver releases gamepiece
      m_armSub.releaseGamePiece();
  } else if (m_driveController.getAButtonReleased()) { //driver unreleases gamepiece
      m_armSub.unreleaseGamepiece();
  } else if (m_operatorController.getRawButton(10)) { //should be middle click on operator 
      m_armSub.hardCenter1(1.0); 

  }else if (m_operatorController.getRawButtonReleased(10)) { //should be middle click on operator 
          m_armSub.hardCenter1(0);
  } else if (m_driveController.getRightBumper()) {
      m_armSub.intakeConePointOut();
  } else if (m_driveController.getLeftBumper()) {
      m_armSub.intakeConePointIn();
  } else if (m_driveController.getStartButton()) {
      m_armSub.intakeCube();
  }else if (m_driveController.getStartButtonReleased()) {
      m_armSub.stopIntakingCube();
  } else if (m_driveController.getPOV() == 180) {
      m_armSub.levelOne();
  } else if (m_driveController.getPOV() == 270) {
      m_armSub.levelTwo();
  }  else if (m_driveController.getPOV() == 0) {
      m_armSub.levelThree();
  }else if (m_operatorController.getPOV() == 0) {
      m_armSub.intakeCubeHP();
  }else if (m_operatorController.getPOV() == 180) {
      m_armSub.intakeConeHP();
  }else if (m_operatorController.getPOV() == 90) {
        m_armSub.intakeCubeHPDoubleStation(true);
        HPDoubleCubeReleaseReset = true;
  }else if (m_operatorController.getPOV() == -1 && HPDoubleCubeReleaseReset) {
        m_armSub.intakeCubeHPDoubleStation(false);
        HPDoubleCubeReleaseReset = false;
  }else if (m_operatorController.getPOV() == 270) {
        m_armSub.intakeConeHPDoubleStation(true);
        HPDoubleConeReleaseReset = true;
    }else if (m_operatorController.getPOV() == -1 && HPDoubleConeReleaseReset) {
        m_armSub.intakeConeHPDoubleStation(false);
        HPDoubleConeReleaseReset = false;
  }//else if (m_operatorController.getPOV() == 270) {
   //     m_armSub.chargeStationArmDown();}

  if (Math.abs(m_operatorController.getLeftY()) > 0.1){
      m_armSub.liveAdjustDistal(m_operatorController.getLeftY());
  }
  if (Math.abs(m_operatorController.getRightY()) > 0.1){
      m_armSub.liveAdjustProxal(m_operatorController.getRightY());
  }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
