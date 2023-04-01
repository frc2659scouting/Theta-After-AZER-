package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmState;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private static CANSparkMax m_armBase1 = new CANSparkMax(Constants.ARM_MOTOR_1, MotorType.kBrushless);
    private static CANSparkMax m_armBase2 = new CANSparkMax(Constants.ARM_MOTOR_2, MotorType.kBrushless);
    private static CANSparkMax m_armUpper1 = new CANSparkMax(Constants.ARM_MOTOR_3, MotorType.kBrushless);

    private static SparkMaxPIDController m_pidBase;
    private static SparkMaxPIDController m_pidUpper;
    private static RelativeEncoder m_encoderBase;

    private static double startTime;
    private static double currentTime = Timer.getFPGATimestamp();

    private static DigitalInput m_hall0 = new DigitalInput(0);

    private static RelativeEncoder m_encoderUpper;

    private static VictorSPX m_endEffectorVictor = new VictorSPX(17);

    public static PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
    /*
        private static final int PH_CAN_ID = 20;
        private static final int SOLENOID_CHANNEL = 8;
        PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
        Solenoid m_intakeSolenoid = m_pH.makeSolenoid(SOLENOID_CHANNEL);
    */
    //`    private static Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 8);
    //    private static final Solenoid m_intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    //    private static    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private static int gamePiece = 0; //0 - no gamepiece, 1 - coob, 2 - cone pointed in, 3 - cone pointed out, 4 - tip grab
    private static int levelScored = 0;
    private static double intakingPower = 0.6; //was .55 3/8/23
    private static double holdingPower = 0.2; // was 16
    private static double centeringPower = 0.9;
    private static double filteredRetractionCurrent;

    private static boolean intook = false;

    private static double retractCurrentThreshold = 10;//previously 8, high value = turned off
    private static double current5 = m_pdh.getCurrent(5);
    //SmartDashboard.putNumber("Current Channel 5", current5);


    private static double bounceTime = 0.4; // 1 / N bounces per second

    private static double hardCenterTimeStamp = 0;

    private static double spitPower = 0.4;
    private static double tipOutSpitPower = 0.2; //was 0.15
    private static double cubeSpitPower = 0.5;
    
    private static double distalAccel = 50;
    private static double distalMaxVel = 20;
    private static double distalT0 = 0;
    private static double distalT1 = 0;
    private static double distalT2 = 0;
    private static double distalT3 = 0;
    private static double distalx = 0;
    private static double distalx0 = 0;
    private static double distalx1 = 0;
    private static double distalx2 = 0;
    private static double distalx3 = 0;
    
    private static double proxalAccel = 20;
    private static double proxalMaxVel = 20;
    private static double proxalT0 = 0;
    private static double proxalT1 = 0;
    private static double proxalT2 = 0;
    private static double proxalT3 = 0;
    private static double proxalx = 0;
    private static double proxalx0 = 0;
    private static double proxalx1 = 0;
    private static double proxalx2 = 0;
    private static double proxalx3 = 0;

    private static boolean scored = false;
    private static boolean endEffectorSolenoid = false;
    public static boolean autonomousSolenoid = false;
    private static boolean hardCentering = false;
    private static boolean stopCentering = false;
    private static boolean stopIntaking = false;
    private static boolean homed = true;
    private static boolean homing = false;
    private static boolean sensorOverride = false;
    private static boolean forceTraverse = false; //forces arm to go to traversal point when homing with tip in or cube
    public static boolean autonomous = true;

    private static ArmState m_selectedState;
    private static ArmState m_previousState;//for motion profiling

    private static ArmState m_home = new ArmState(5, 5, false); //was 0,0 - changed to avoid flange of cone catching
    //    private static ArmState m_intakePointIn = new ArmState(Constants.INTAKING_POSITION_P, Constants.INTAKING_POSITION_D, true);
    private static ArmState m_intakePointOut = new ArmState(26.75, -20.0, true); //28, -20 | 2-15-23 was 25.5, -20, t |
    private static ArmState m_intakePointOutBounce = new ArmState(25.5, -20.0, true); //3-11-2023 high gain bounce (26.3, -20.0)
    private static ArmState m_intakePointIn = new ArmState(26.7, -27.5, false); //28, -28 | 2-15-23 26,-29
    private static ArmState m_intakePointInAuto = new ArmState(26.5, -25.5, false); //28, -28 | 2-15-23 26,-29
    private static ArmState m_intakePointInBounce = new ArmState(25.5, -27.5, false); //3-11-2023 high gain bounce (26.2, -27.5)
    private static ArmState m_intakeCubeHP = new ArmState(-8.25, 9.65, true); //Single station
    private static ArmState m_intakeCubeHPDouble = new ArmState(-9.07, 31.6, false); //Single station
    private static ArmState m_intakeCubeHPDoubleDown = new ArmState(-9.07, 31.6, false); //Single station
    private static ArmState m_intakeConeHP = new ArmState(-5.2, 10.5, true);  //Single station
    private static ArmState m_intakeConeHPDouble = new ArmState(10.1, -44.2-.5, true);  //Single station
    private static ArmState m_intakeConeHPDoubleDown = new ArmState(10.85, -42.0, true);  //Single station
    private static ArmState m_intakeCube = new ArmState(27.75, -31, false); //29, -30 |2-15-23 27, -33
    private static ArmState m_intakeCubeBounce = new ArmState(25, -31, false); //29, -30
    private static ArmState m_intakeCubeBounceAuto = new ArmState(20, -31, false);

    private static ArmState m_intakeTraverse = new ArmState(17, -20, true); // was 15, -25

    private static ArmState m_intakeTraverse1 = new ArmState(12, -12, true); // was 12, -12

    private static ArmState m_levelOne = new ArmState(-10, 10, false);
    private static ArmState m_levelOneTipOut = new ArmState(-10, 10, true);
    //private static ArmState m_levelOneTipOut = new ArmState(-10, 10, false);

    private static ArmState m_preScore = new ArmState(10, 30  , true);
    private static ArmState m_preScoreAuto = new ArmState(10, 30  , false);


    private static ArmState m_levelTwoTipOut = new ArmState(-11, 36.5, false); //2-18 -11, 36, false || 2-15-23 was -8,34,f |
    private static ArmState m_levelTwoTipOutRelease = new ArmState(-14, 35.5, false); //2-15-23 was -10,34,f
    private static ArmState m_levelThreeTipOut = new ArmState(31.75, -72, true);
    private static ArmState m_levelThreeTipOutRelease = new ArmState(31.75, -72, true);

    private static ArmState m_levelTwoTipIn = new ArmState(-9.3, 30.0, true); //3-27 was -9,29,true
    private static ArmState m_levelTwoTipInRelease = new ArmState(-13.5, 23, true); //3-17 was -13, 23, true
    private static ArmState m_levelThreeTipIn = new ArmState(-15.8, 52.5, false);//-14, 49.5, false
    private static ArmState m_levelThreeTipInRelease = new ArmState(-16, 45, false);

    private static ArmState m_levelTwoTipGrab = new ArmState(-6.6, 29.3, false);
    private static ArmState m_levelThreeTipGrab = new ArmState(-12, 46, false);

    private static ArmState m_autoIntakeTraverse = new ArmState(17, -20, false);
    private static ArmState m_levelTwoCube = new ArmState(-5, 25, false);
    private static ArmState m_levelThreeCube = new ArmState(-10, 40, false);
    //private static ArmState m_levelThreeCubeRelease = new ArmState(-15, 55, false);
    private static ArmState m_chargeStation = new ArmState(30, -25.0, true);
    private static double distalAdjust = 0;
    private static double proxalAdjust = 0;
    
    public void initialize() {
    m_armBase1.clearFaults();
    m_armBase1.restoreFactoryDefaults();
    m_armBase2.clearFaults();
    m_armBase2.restoreFactoryDefaults();
    m_armUpper1.clearFaults();
    m_armUpper1.restoreFactoryDefaults();
    m_armBase1.setInverted(true);
    m_armBase2.follow(m_armBase1,true);    
    m_armBase2.setIdleMode(IdleMode.kBrake);
    m_armBase1.setIdleMode(IdleMode.kBrake);
    m_armBase1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armBase1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armBase1.setSoftLimit(SoftLimitDirection.kForward,30);
    m_armBase1.setSoftLimit(SoftLimitDirection.kReverse,-30);

    // set PID coefficients
    m_armUpper1.setIdleMode(IdleMode.kBrake);
    m_armUpper1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armUpper1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armUpper1.setSoftLimit(SoftLimitDirection.kForward,60);
    m_armUpper1.setSoftLimit(SoftLimitDirection.kReverse,-80);

    m_pidUpper = m_armUpper1.getPIDController();
    m_encoderUpper = m_armUpper1.getEncoder();
    m_pidUpper.setP(0.05);//AZER was 0.04
    m_pidUpper.setI(0.0);
    m_pidUpper.setD(0.0);
    m_pidUpper.setIZone(0.0);
    m_pidUpper.setFF(0.0);
    m_pidUpper.setOutputRange(-0.65, 0.65);
    
    int smartMotionSlot = 0;
    m_pidUpper.setSmartMotionMaxVelocity(1, smartMotionSlot);
    m_pidUpper.setSmartMotionMinOutputVelocity(-1, smartMotionSlot);
    m_pidUpper.setSmartMotionMaxAccel(10, smartMotionSlot);
    m_pidUpper.setSmartMotionAllowedClosedLoopError(0.01, smartMotionSlot);

    m_pidBase = m_armBase1.getPIDController();
    m_encoderBase = m_armBase1.getEncoder();

    m_pidBase.setP(.075);
    m_pidBase.setI(0.0);
    m_pidBase.setD(0.0);
    m_pidBase.setIZone(0.0);
    m_pidBase.setFF(0.0);
    m_pidBase.setOutputRange(-0.6, 0.6);

        //        m_pidUpper.setP(Constants.ARM_KP_UPPER);
        //        m_pidUpper.setI(0.0);
        //        m_pidUpper.setD(Constants.ARM_KD_UPPER);
        m_endEffectorVictor.configOpenloopRamp(0.25);

        m_selectedState = m_home;
    }

    public void setAutoGamePiece(int i) {
        gamePiece = i;
    }
    
    public void chargeStationArmDown() {
        m_selectedState = m_chargeStation;
    }
    
    public boolean readHall0() {
        SmartDashboard.putBoolean("Hall Sensor 0", m_hall0.get());
        return m_hall0.get();
    }

    public void setSol(Boolean autoSol) {
        autonomousSolenoid = autoSol;
//        SmartDashboard.putBoolean("autonomousSolenoid", autonomousSolenoid);
    }
    public boolean getSolAuto() {
//            SmartDashboard.putBoolean("autonomousSolenoid", autonomousSolenoid);
        return autonomousSolenoid;    }    

    public void goHome() {
        distalAdjust = 0;
        proxalAdjust = 0;
        homing = true;
        if (!scored) {
            /*if (gamePiece == 0) m_endEffectorVictor.set(ControlMode.PercentOutput, 0);
            else if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            else if (gamePiece == 2) m_endEffectorVictor.set(ControlMode.PercentOutput, holdingPower);
            else if (gamePiece == 3) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            */
            hold();
            if (gamePiece == 1 || gamePiece == 2) {
                if((((!m_hall0.get()|| sensorOverride) && (!forceTraverse || Math.abs(m_encoderBase.getPosition() - m_intakeTraverse.getProximal()) < .3 )&& (!forceTraverse || Math.abs(m_encoderUpper.getPosition() - m_intakeTraverse.getDistal()) < .3 )  ) && m_selectedState==m_intakeTraverse) || (m_selectedState == m_intakeConeHP || m_selectedState == m_intakeCubeHP)){
                    m_selectedState = m_home;
                    homed = true;
                    homing = false;
                }
                else m_selectedState = m_intakeTraverse;
            } else {
                m_selectedState = m_home;
                homed = true;
                homing = false;
            } 
        } else { //scored (no gamepiece)
            //homed = false;
            m_endEffectorVictor.set(ControlMode.PercentOutput, 0);
            if(levelScored == 3 || levelScored == 2 && gamePiece == 3){
                if(Math.abs(m_encoderBase.getPosition() - m_preScore.getProximal()) < 5 && Math.abs(m_encoderUpper.getPosition() - m_preScore.getDistal()) < 5 ){
                    m_selectedState = m_home;
                    homing = false;
                    scored = false;
                }
                else m_selectedState = m_preScore;
            }
            else m_selectedState = m_home;
            //homed = true;
            //homing = false;
        }

    }
    public void traverse() {
        m_selectedState = m_intakeTraverse;
        if (!scored) {
        hold();
        /*    
            if (gamePiece == 0) m_endEffectorVictor.set(ControlMode.PercentOutput, 0);
            else if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            else if (gamePiece == 2) m_endEffectorVictor.set(ControlMode.PercentOutput, holdingPower);
            else if (gamePiece == 3) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
        */
        }
    }

    public void intakeCubeAuto() {
        if (!intook) m_selectedState = m_intakeCubeBounceAuto;
        else m_selectedState = m_intakeCube;

        m_pidBase.setReference(m_selectedState.proximal + 3, CANSparkMax.ControlType.kPosition);
        m_pidUpper.setReference(m_selectedState.distal, CANSparkMax.ControlType.kPosition);

        m_endEffectorVictor.set(ControlMode.PercentOutput, -0.85);
        gamePiece = 2;

        intook = true;
    }

    public void intakeConeHPDoubleStation(Boolean i) {
        homed = true;
        homing = false;
        gamePiece = 2;
        scored = false;
            if(i){
                m_selectedState = m_intakeConeHPDoubleDown;
                m_endEffectorVictor.set(ControlMode.PercentOutput, intakingPower);
            }
            else {
                m_selectedState = m_intakeConeHPDouble;
                hold();
            }
    }
    public void intakeCubeHPDoubleStation(Boolean up) {
        homed = true;
        homing = false;
        gamePiece = 1;
        scored = false;
            if(up){
                m_selectedState = m_intakeCubeHPDoubleDown;
                m_endEffectorVictor.set(ControlMode.PercentOutput, -intakingPower);
            }
            else {
                m_selectedState = m_intakeCubeHPDouble;
                hold();
            }
    }

    public void prescoreAuto() {
        m_selectedState = m_preScoreAuto;

        m_pidBase.setReference(m_selectedState.proximal, CANSparkMax.ControlType.kPosition);
        m_pidUpper.setReference(m_selectedState.distal, CANSparkMax.ControlType.kPosition);
        setSol(m_selectedState.pneumaticPos);
        gamePiece = 2;
    }

    public void autoPrepare() {
        intook = false;
    }

    public void prescore() {
        m_selectedState = m_preScore;
    }
    
    public void stopHoming() {
        homing = false;
        homed = true;
    }

    public void startIntaking() {
        stopIntaking = false;
    }

    public void stopIntaking() {
        stopIntaking = true;
    }

    public void intakeCube() {
        homed = false;
        homing = false;
        //        m_selectedState = m_intakeCube;
        if (!stopIntaking) {
            //if (Timer.getFPGATimestamp() % bounceTime > bounceTime / 2) m_selectedState = m_intakeCubeBounce;
            //else m_selectedState = m_intakeCube;
            m_selectedState = m_intakeCube;
            m_endEffectorVictor.set(ControlMode.PercentOutput, -intakingPower);
            gamePiece = 1;
            scored = false;
        }
        else m_selectedState = m_intakeCube;

    }
    public void stopIntakingCube() {
        if (m_selectedState == m_intakeCube){
        homed = false;
        //        m_selectedState = m_intakeCube;
        m_selectedState = m_intakeCubeBounce;
            m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
           
        }
    }

    public void intakeCubeHP() {
        homed = true;
        homing = false;
        //        m_selectedState = m_intakeCube;
        //if (!stopIntaking) {
            m_selectedState = m_intakeCubeHP;
            m_endEffectorVictor.set(ControlMode.PercentOutput, -intakingPower);
            gamePiece = 1;
            scored = false;
        //}
//        else m_selectedState = m_intakeCube;

    }public void intakeConeHP() {
        homed = true;
        homing = false;
        //        m_selectedState = m_intakeCube;
        //if (!stopIntaking) {
            m_selectedState = m_intakeConeHP;
            m_endEffectorVictor.set(ControlMode.PercentOutput, intakingPower);
            gamePiece = 2;
            scored = false;
        //}

    }
    public void liveAdjustDistal(double distalAdjustIn) {
        distalAdjust = distalAdjust - distalAdjustIn/3; // was /10
    } 

    public void liveAdjustProxal(double proxalAdjustIn) {
        proxalAdjust = proxalAdjust - proxalAdjustIn/3;
    } 
    
    public void intakeConePointIn() {
        homed = false;
        homing = false;
        if (!stopIntaking) {
            if (Timer.getFPGATimestamp() % bounceTime > bounceTime / 2 ) m_selectedState = m_intakePointInBounce;
            else m_selectedState = m_intakePointIn;
            m_endEffectorVictor.set(ControlMode.PercentOutput, intakingPower);
            gamePiece = 2;
            scored = false;
            //go to intake pos
            //claw to cone position
        }
    }

    public void traverseAuto() {
        homed = true;
        homing = false;
        m_selectedState = m_autoIntakeTraverse;
        proxalAdjust = 0.0;
        m_endEffectorVictor.set(ControlMode.PercentOutput, 0.85);
        gamePiece = 2;
        scored = false;
        //go to intake pos
        //claw to cone position
    }

    public void intakeConePointInAuto() {
        homed = false;
        homing = false;
        m_selectedState = m_intakePointInAuto;
        proxalAdjust = 0.0;
        m_endEffectorVictor.set(ControlMode.PercentOutput, 0.85);
        gamePiece = 2;
        scored = false;
        //go to intake pos
        //claw to cone position
    }
    
    public void intakeConePointOut() {
        homed = false;
        homing = false;
        //        m_selectedState = m_intakePointOut;
        if (!stopIntaking) {
            if (Timer.getFPGATimestamp() % bounceTime > bounceTime / 2) m_selectedState = m_intakePointOutBounce;
            else m_selectedState = m_intakePointOut;
            m_endEffectorVictor.set(ControlMode.PercentOutput, -intakingPower);
            gamePiece = 3;
            scored = false;
            //go to intake pos
            //claw to cone position
        }
    }
    public void levelOne() {
        levelScored = 1;
        hold();
        if (gamePiece == 1 || gamePiece == 2) {
            m_selectedState = m_levelOne;
        }
        else m_selectedState = m_levelOneTipOut;
        //elevation mechanism to level 1
    }
    public void levelTwo() {
        levelScored = 2;
        hold();
        if (homed) {
            if (gamePiece == 1) m_selectedState = m_levelTwoCube;
            else if (gamePiece == 2) m_selectedState = m_levelTwoTipIn;
            else if (gamePiece == 3) m_selectedState = m_levelTwoTipOut;
            else if (gamePiece == 4) m_selectedState = m_levelTwoTipGrab;
        }
        //elevation mechanism to level 2
    }

    public void levelThree() {
        levelScored = 3;
        hold();
        if (autonomous || homed) {
            if (gamePiece == 1) m_selectedState = m_levelThreeCube;
            else if (gamePiece == 2) m_selectedState = m_levelThreeTipIn;
            else if (gamePiece == 3) m_selectedState = m_levelThreeTipOut;
            else if (gamePiece == 4) m_selectedState = m_levelThreeTipGrab;
        }
        //big long thingy to level 3 scoring pos
    }

    public void hold() {
        if(!hardCentering){
            if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            else if (gamePiece == 2) {
                m_endEffectorVictor.set(ControlMode.PercentOutput, holdingPower);
            } else if (gamePiece == 3) {
                m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            } else if (gamePiece == 4) {
                m_endEffectorVictor.set(ControlMode.PercentOutput, holdingPower);
            }
        }
    }

    public void releaseGamePiece() {
        if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, cubeSpitPower);
        else if (gamePiece == 2) {
            m_endEffectorVictor.set(ControlMode.PercentOutput, -spitPower);
            if (m_selectedState == m_levelTwoTipIn ||m_selectedState == m_levelTwoTipInRelease ) {
                m_selectedState = m_levelTwoTipInRelease;
            }
            else if (m_selectedState == m_levelThreeTipIn || m_selectedState == m_levelThreeTipInRelease) {
                m_selectedState = m_levelThreeTipInRelease;
            }
            else m_endEffectorVictor.set(ControlMode.PercentOutput, -spitPower);
        } else if (gamePiece == 3) {
        
            if (m_selectedState == m_levelTwoTipOut ||m_selectedState == m_levelTwoTipOutRelease) {
                m_selectedState = m_levelTwoTipOutRelease;
                m_endEffectorVictor.set(ControlMode.PercentOutput, tipOutSpitPower);
            }
            else if (m_selectedState == m_levelThreeTipOut || m_selectedState == m_levelThreeTipOutRelease ) {
                m_selectedState = m_levelThreeTipOutRelease;
                m_endEffectorVictor.set(ControlMode.PercentOutput, spitPower);    }
            else m_endEffectorVictor.set(ControlMode.PercentOutput, spitPower);
        } else if (gamePiece == 4){
            m_endEffectorVictor.set(ControlMode.PercentOutput, -spitPower);
        }

        scored = true;
        //gamePiece = 0; //this could cause issues with a bad release
        //
    }
    public void unreleaseGamepiece() {
        hold();
        if (gamePiece == 2) {
            if (m_selectedState == m_levelTwoTipInRelease) {
                m_selectedState = m_levelTwoTipIn;
            }
            if (m_selectedState == m_levelThreeTipInRelease) {
                m_selectedState = m_levelThreeTipIn;
            }
        } else if (gamePiece == 3) {
            if (m_selectedState == m_levelTwoTipOutRelease) {
                m_selectedState = m_levelTwoTipOut;
            }
            if (m_selectedState == m_levelThreeTipOutRelease) {
                m_selectedState = m_levelThreeTipOut;
            }
        }
    }

    public boolean solenoidPosition() {
        //if(autonomous) return autonomousSolenoid;
        //else 
        return endEffectorSolenoid;
    }

    public void l3Auto() {
        switch (gamePiece) {
            case 1:
                m_selectedState = m_levelThreeCube;
            break;

            case 2: 
                m_selectedState = m_levelThreeTipIn;
            break;
        }
    }

    public void hardCenter() {
        hardCentering = true;
    }
    public void hardCenter1(double power1) {
        if(gamePiece != 2){
        m_endEffectorVictor.set(ControlMode.PercentOutput, -power1);}
        else m_endEffectorVictor.set(ControlMode.PercentOutput, power1);
    }
    /*    public void releaseConeTipOut() {
           m_endEffectorVictor.set(ControlMode.PercentOutput, -0.5);
           //gamePiece = 0; //this could cause issues with a bad release
           //
       }*/

    public static double[] armAngles() {
        double[] angles = {
            m_selectedState.getProximal(), 
            m_selectedState.getDistal()
        };

        return angles;
    }
    public void endEffectorOpenLoop (double power){
        m_endEffectorVictor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        //        m_pidBase.setReference(m_intakePointIn.proximal, CANSparkMax.ControlType.kPosition);
        //        m_pidUpper.setReference(m_intakePointIn.getUpper()*m_encoderBase.getPosition(), CANSparkMax.ControlType.kPosition);
        readHall0();

        if (homing) {
            goHome();
        }

        if (hardCentering) {
            if (gamePiece == 0) m_endEffectorVictor.set(ControlMode.PercentOutput, 0);
            else if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, -centeringPower);
            else if (gamePiece == 2) m_endEffectorVictor.set(ControlMode.PercentOutput, centeringPower);
            else if (gamePiece == 3) m_endEffectorVictor.set(ControlMode.PercentOutput, -centeringPower);
            hardCentering = false;
            stopCentering = true;
        } else if (stopCentering) {
            stopCentering = false;
            hold();
            /*
            if (gamePiece == 0) m_endEffectorVictor.set(ControlMode.PercentOutput, 0);
            else if (gamePiece == 1) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);
            else if (gamePiece == 2) m_endEffectorVictor.set(ControlMode.PercentOutput, holdingPower);
            else if (gamePiece == 3) m_endEffectorVictor.set(ControlMode.PercentOutput, -holdingPower);*/
        }
        //auto intake retract
        current5 = m_pdh.getCurrent(5); // checks intake current
        SmartDashboard.putNumber("Current Channel 5", current5);

        filteredRetractionCurrent = Math.abs(current5) * .15 + filteredRetractionCurrent * .85;
        SmartDashboard.putNumber("Intake Filtered Current", filteredRetractionCurrent);

        if (!autonomous && Math.abs(filteredRetractionCurrent) > retractCurrentThreshold && (m_selectedState == m_intakeConeHP ||m_selectedState == m_intakeCubeHP || m_selectedState == m_intakeCube || m_selectedState == m_intakePointIn || m_selectedState == m_intakePointOut)) { //absolute value
            stopIntaking();
            goHome();
        }
        // end auto retract intake
        
        //motion control code
        /*if(m_previousState != m_selectedState){
            distalT0 = Timer.getFPGATimestamp();
            distalx0 = m_encoderUpper.getPosition();
            distalx3 = m_selectedState.getDistal();
            distalT1 = distalMaxVel / distalAccel + distalT0;
            distalx1 = distalx0 + 0.5 * distalAccel * Math.pow((distalT1 - distalT0),2);
            distalx2 = distalx3 - (distalx1 - distalx0);
            distalT2 = (distalx2-distalx1)/distalMaxVel + distalT1;
            distalT3 = distalT2 + (distalT1-distalT0);
            if(distalT3 - distalT0 < (distalT1 - distalT0)*2){ //takes care of the condition where we cant achieve full speed
                distalT1 = (distalT3 - distalT0)/2 + distalT0;
                distalT2 = distalT1;
            }
        }
        
        if(Timer.getFPGATimestamp()  < distalT1){
            distalx = 0.5 * distalAccel * Math.pow((Timer.getFPGATimestamp() - distalT0), 2);
        }else if(Timer.getFPGATimestamp()  < distalT2){
            distalx = distalx1 + (Timer.getFPGATimestamp() - distalT0)*distalMaxVel;
        }else if(Timer.getFPGATimestamp()  < distalT3){
            distalx = distalx2 + (Timer.getFPGATimestamp() - distalT2) * (distalMaxVel - 0.5 * distalAccel * Math.pow((Timer.getFPGATimestamp() - distalT0), 2));
        }else{
            distalx = m_selectedState.getDistal();
            //set m_SelectedState
        }

        if(Timer.getFPGATimestamp()  < proxalT1){
            proxalx = 0.5 * proxalAccel * Math.pow((Timer.getFPGATimestamp() - proxalT0), 2);
        }else if(Timer.getFPGATimestamp()  < proxalT2){
            proxalx = proxalx1 + (Timer.getFPGATimestamp() - proxalT0)*proxalMaxVel;
        }else if(Timer.getFPGATimestamp()  < proxalT3){
            proxalx = proxalx2 + (Timer.getFPGATimestamp() - proxalT2) * (proxalMaxVel - 0.5 * proxalAccel * Math.pow((Timer.getFPGATimestamp() - proxalT0), 2));
        }else{
            proxalx = m_selectedState.getProximal();
            //set m_SelectedState
        }
        SmartDashboard.putNumber("distalMotionControlCommandPosition",distalx);
        SmartDashboard.putNumber("distalx0",distalx0);
        SmartDashboard.putNumber("distalx1",distalx1);
        SmartDashboard.putNumber("distalx2",distalx2);
        SmartDashboard.putNumber("distalx3",distalx3);
        SmartDashboard.putNumber("distalT0",distalT0);
        SmartDashboard.putNumber("distalT1",distalT1);
        SmartDashboard.putNumber("distalT2",distalT2);
        SmartDashboard.putNumber("distalT3",distalT3);

        m_previousState = m_selectedState;
        */ 
        //end motion control code

        
        //m_pidBase.setSmartMotionMaxAccel(m_accel.getBaseAccel(m_encoderBase.getPosition(), m_encoderUpper.getPosition()), 0);
        //m_pidUpper.setSmartMotionMaxAccel(m_accel.getUpperAccel(m_encoderBase.getPosition(), m_encoderUpper.getPosition()), 0);

        //      Endeffector Pneumatic logic
        if (m_selectedState == m_home) {

            if (gamePiece == 1) endEffectorSolenoid = false;
            else if (gamePiece == 2) endEffectorSolenoid = false;
            else if (gamePiece == 3) endEffectorSolenoid = true;
        } else {
            if (m_selectedState.pneumaticPos) endEffectorSolenoid = false;
            else endEffectorSolenoid = true;
        }

        m_pidBase.setReference(m_selectedState.proximal + proxalAdjust, CANSparkMax.ControlType.kPosition);
        m_pidUpper.setReference(m_selectedState.distal + distalAdjust, CANSparkMax.ControlType.kPosition);
        
        //        if(m_selectedState.conePointIn) m_intakeSolenoid.set(true);
        //            else m_intakeSolenoid.set(false);
        //        if(m_selectedState.conePointIn) endEffectorSolenoid = false;
        //        else endEffectorSolenoid = true;

        SmartDashboard.putNumber("Base Arm", m_encoderBase.getPosition());
        SmartDashboard.putNumber("Upper Arm", m_encoderUpper.getPosition());
        SmartDashboard.putNumber("Game Piece", gamePiece);
        SmartDashboard.putString("Selected Arm State", m_selectedState.toString()); // Might not work

        currentTime = Timer.getFPGATimestamp();

        if (currentTime - startTime > 250) {
            startTime = Timer.getFPGATimestamp();
            DataLogManager.log(
                "Arm state : " + m_selectedState.toString() + "\n" +
                "Distal : " + m_selectedState.getDistal() + "\n" +
                "Proximal : " + m_selectedState.getProximal() + "\n" +
                "End effector pos : " + m_selectedState.getEndEffectorPos() + "\n" +
                "Intake Current : " + current5 + "\n" +
                "Game piece : " + gamePiece + "\n" +
                "Hall sensor : " + readHall0()
            );
        }
    }
}