package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a
 * {@link RoboticsAPITask#run()} method, which will be called successively in
 * the application lifecycle. The application will terminate automatically after
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an
 * exception is thrown during initialization or run.
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the
 * {@link RoboticsAPITask#dispose()} method.</b>
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotApplication extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool gripper;
	
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getRobot(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		gripper = getApplicationData().createFromTemplate("SchunkSDH2");
		gripper.attachTo(lbr_iiwa_14_R820_1.getFlange());
	}

	public void run() {
	CartesianImpedanceControlMode cartImpCtrlMode = new CartesianImpedanceControlMode();
	cartImpCtrlMode.parametrize(CartDOF.X,	CartDOF.Y).setStiffness(3000.0);
	cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(1.0);
	cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300.0);
	cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(0.7);
	lbr_iiwa_14_R820_1.move( lin (getApplicationData().getFrame("/P1")).setCartVelocity(800).setMode(cartImpCtrlMode)); //.setCartVelocity(800)
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		RobotApplication app = new RobotApplication();
		app.runApplication();
	}
}
