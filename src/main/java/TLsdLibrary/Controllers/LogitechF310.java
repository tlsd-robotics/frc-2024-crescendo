package TLsdLibrary.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** 
 * <p>Creates a Logitech F310 specific {@link Joystick} including methods for every button and axis</p>
 */
public class LogitechF310 extends GenericController {

//==============================================================================
//=============================== Joystick IDs ===================================
    
    private final int btnA = 1;
    private final int btnB = 2;
    private final int btnX = 3;
    private final int btnY = 4;
    private final int btnLB = 5;
    private final int btnRB = 6;
    private final int btnBack = 7;   
    private final int btnStart = 8;       
    private final int btnL3 = 9;
    private final int btnR3 = 10;
    public final int leftXAxis = 0;
    public final int leftYAxis = 1;
    public final int leftTrigger = 2;
    public final int rightTrigger = 3;
    public final int rightXAxis = 4;
    public final int rightYAxis = 5;

//==============================================================================
//============================== Constructor ===================================

    /**
     * Create Logitech F310 Gamepad object
     * @param port
     */
    public LogitechF310(int portID) {
        super(portID);
    }

//==============================================================================
//============================ Create Buttons ==================================

    public JoystickButton buttonA     = new JoystickButton(joy, btnA);
    public JoystickButton buttonB     = new JoystickButton(joy, btnB);
    public JoystickButton buttonX     = new JoystickButton(joy, btnX);
    public JoystickButton buttonY     = new JoystickButton(joy, btnY);
    public JoystickButton buttonLB    = new JoystickButton(joy, btnLB);
    public JoystickButton buttonRB    = new JoystickButton(joy, btnRB);
    public JoystickButton buttonBack  = new JoystickButton(joy, btnBack);
    public JoystickButton buttonStart = new JoystickButton(joy, btnStart);
    public JoystickButton buttonR3    = new JoystickButton(joy, btnR3);
    public JoystickButton buttonL3    = new JoystickButton(joy, btnL3);

    public POVButton dPadUp    = new POVButton(joy, 0);
    public POVButton dPadRight = new POVButton(joy, 90);
    public POVButton dPadDown  = new POVButton(joy, 180);
    public POVButton dPadLeft  = new POVButton(joy, 270);

//==============================================================================
//================================ Getters =====================================
    /**
     * 
     * @return Gamepad Object
     */
    public Joystick getJoy() {
        return joy;
    }

    /**
     * 
     * @return Returns A button object
     */
    public JoystickButton getButtonA() {
        return buttonA;
    }

    /**
     * 
     * @return Returns B button Object
     */
    public JoystickButton getButtonB() {
        return buttonB;
    }

    /**
     * 
     * @return Returns X button object
     */
    public JoystickButton getButtonX() {
        return buttonX;
    }

    /**
     * 
     * @return Returns Y button object
     */
    public JoystickButton getButtonY() {
        return buttonY;
    }

    /**
     * 
     * @return Returns Left bumper button object
     */
    public JoystickButton getButtonLB() {
        return buttonLB;
    }

    /**
     * 
     * @return Returns right bumper button object
     */
    public JoystickButton getButtonRB() {
        return buttonRB;
    }

    /**
     * 
     * @return Return select button object
     */
    public JoystickButton getButtonStart() {
        return buttonStart;
    }

    /**
     * 
     * @return Returns Right joystick click button object
     */
    public JoystickButton getButtonR3() {
        return buttonR3;
    }

    /**
     * 
     * @return Returns Left joystick click button object
     */
    public JoystickButton getButtonL3() {
        return buttonL3;
    }

    /**
     * 
     * @return Returns D-PAD Up button object
     */
    public POVButton getDPadUp() {
        return dPadUp;
    }

    /**
     * 
     * @return Returns D-PAD Right button object
     */
    public POVButton getDPadRight() {
        return dPadRight;
    }

    /**
     * 
     * @return Returns D-PAD Down button
     */
    public POVButton getDPadDown() {
        return dPadDown;
    }

    /**
     * 
     * @return Returns D-PAD Left Button
     */
    public POVButton getDPadLeft() {
        return dPadLeft;
    }

    /**
     * 
     * @return Returns Leftside X axis joystick value
     */
    public double getLeftXAxis() {
        return joy.getRawAxis(leftXAxis);
    }

    /**
     * 
     * @return Returns Leftside Y axis joystick value
     */
    public double getLeftYAxis() {
        return joy.getRawAxis(leftYAxis);
    }

    /**
     * 
     * @return Returns Rightside X axix joystick value
     */
    public double getRightXAxis() {
        return joy.getRawAxis(rightXAxis);
    }

    /**
     * 
     * @return Returns Rightside Y Axis joystick value
     */
    public double getRightYAxis() {
        return joy.getRawAxis(rightYAxis);
    }

    /**
     * 
     * @return Returns left trigger button value. <hr> For future reference, create a button form of the trigger for that mode
     */
    public double getLeftTrigger() {
        return joy.getRawAxis(leftTrigger);
    }

    /**
     * 
     * @return Returns Right trigger button value. <hr> For future reference, create a button form of the trigger for that mode
     */
    public double getRightTrigger() {
        return joy.getRawAxis(rightTrigger);
    }

}