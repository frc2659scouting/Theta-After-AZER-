package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    enum AutonMode {
        JUST_SCORE, LEFT_2_PIECE_CUBE, DO_NOTHING, RIGHT_2_PIECE_BAL, LEFT_2_PIECE_BAL, RIGHT_2_PIECE_CUBE, JUST_DRIVE_BACK, SCORE_BALANCE, RIGHT_SIDE_2_PIECE, ALMOST_2_PIECE_RIGHT, LEFT_SIDE_2_PIECE, TUNING_AUTO, ALMOST_2_PIECE_LEFT
    }

    private AutonMode mCachedAutoMode = null;

 //    private SendableChooser<DesiredMode> mModeChooser;
    private static SendableChooser<AutonMode> mAutoMode;
 //   private AutonMode autoModeReturn = null;
    private static String autoChoiceReturn;

    public AutoModeSelector() {
        mAutoMode = new SendableChooser<>();
        mAutoMode.addOption("JUST SCORE", AutonMode.JUST_SCORE); //ID: 1
        mAutoMode.addOption("DO NOTHING", AutonMode.DO_NOTHING); //ID: 2
        mAutoMode.addOption("JUST DRIVE BACK", AutonMode.JUST_DRIVE_BACK); //ID: 3
        mAutoMode.addOption("SCORE AND BALANCE", AutonMode.SCORE_BALANCE); //ID: 4
        mAutoMode.addOption("RIGHT SIDE 2 PIECE", AutonMode.RIGHT_SIDE_2_PIECE); //ID: 5
        mAutoMode.addOption("Right Side 1.5 Piece", AutonMode.ALMOST_2_PIECE_RIGHT); //ID: 6
        mAutoMode.addOption("LEFT SIDE 2 PIECE", AutonMode.LEFT_SIDE_2_PIECE); //ID: 7
        mAutoMode.addOption("TUNING", AutonMode.TUNING_AUTO); //ID: 8
        mAutoMode.addOption("Left Side 1.5 Piece", AutonMode.ALMOST_2_PIECE_LEFT); //ID: 9
        mAutoMode.setDefaultOption("Left 2 Piece + balance", AutonMode.LEFT_2_PIECE_BAL); //ID: 10
        mAutoMode.addOption("Right 2 Piece + balance", AutonMode.RIGHT_2_PIECE_BAL); //ID: 11
        mAutoMode.addOption("Right 2.5 Piece", AutonMode.RIGHT_2_PIECE_CUBE); //ID: 12
        mAutoMode.addOption("Left 2.5 Piece", AutonMode.LEFT_2_PIECE_CUBE); //ID: 13
        SmartDashboard.putData("Auto Mode", mAutoMode);
    }

    public void updateModeCreator() {
        AutonMode desiredMode = mAutoMode.getSelected();
         if (mCachedAutoMode != desiredMode ) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
          //  autoModeReturn = desiredMode;
        }
        mCachedAutoMode = desiredMode;
    }

    public static int returnAutoMode(){
       autoChoiceReturn = mAutoMode.getSelected().toString();
        SmartDashboard.putString(autoChoiceReturn, "autoChoiceReturn");
       switch (autoChoiceReturn) {
            case "JUST_SCORE": 
            return 1;

            case "DO_NOTHING":
            return 2;

            case "JUST_DRIVE_BACK":
            return 3;

            case "SCORE_BALANCE":
            return 4;

            case "RIGHT_SIDE_2_PIECE":
            return 5;

            case "ALMOST_2_PIECE_RIGHT":
            return 6;

            case "LEFT_SIDE_2_PIECE":
            return 7;

            case "TUNING_AUTO":
            return 8;

            case "ALMOST_2_PIECE_LEFT":
            return 9;

            case "LEFT_2_PIECE_BAL":
            return 10;

            case "RIGHT_2_PIECE_BAL":
            return 11;

            case "RIGHT_2_PIECE_CUBE":
            return 12;

            case "LEFT_2_PIECE_CUBE":
            return 13;
       }

       return 0;
    }

    public void outputToSmartDashboard() {
       // SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("Auto Mode Selected", mCachedAutoMode.name());
    }

    public void reset() {
        mCachedAutoMode = null;
    }
}
