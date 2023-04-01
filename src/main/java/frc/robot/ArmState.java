package frc.robot;

public class ArmState {
    
    public double proximal;
    public double distal;
    public boolean pneumaticPos;
    public double dstlToPrmlRatio;

    public ArmState(double prml, double dstl, boolean pneumatic) {
        proximal = prml;
        distal = dstl;
        pneumaticPos = pneumatic;
        dstlToPrmlRatio = distal/proximal; 
    }

    public double getProximal() {
        return proximal;
    }

    public double getDistal() {
        return distal;
    }

    public double getUpper() {
        return dstlToPrmlRatio;
    }

    public boolean getEndEffectorPos() {
        return pneumaticPos;
    }
}