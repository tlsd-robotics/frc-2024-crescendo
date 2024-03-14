package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;




public class Superstructure {
    private final PneumaticHub pneumatics = new PneumaticHub(Constants.Superstructure.PH_CAN_ID);
    private final PowerDistribution pdp = new PowerDistribution(Constants.Superstructure.PDP_CAN_ID, ModuleType.kRev);

    public Superstructure() {}

    public void start() {
        pneumatics.clearStickyFaults();
        pdp.clearStickyFaults();
    }
}