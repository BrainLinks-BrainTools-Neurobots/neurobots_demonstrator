package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.MMCMotions.handGuiding;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.Robot;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;

class RecordedJointState {
	JointPosition pos;
	double time;
}

public class HandGuidingAndReplay extends RoboticsAPIApplication {

	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_14_R820_1;
	private Tool gripper;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_14_R820_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_14_R820_1");
		gripper = getApplicationData().createFromTemplate("SchunkWSG50");
		gripper.attachTo(lbr_iiwa_14_R820_1.getFlange());
	}

	public void run() {
		getLogger().info("to start");
		lbr_iiwa_14_R820_1.setESMState("1");
		// lbr_iiwa_14_R820_1.move(ptp(0, 0, 0, -Math.PI/2, 0, Math.PI/2,
		// 0).setJointVelocityRel(0.2));
		// lbr_iiwa_14_R820_1.move(ptp(-0.81, 0.96, -0.35, -1.46, 1.58, -1.12,
		// 0.04).setJointVelocityRel(0.2));
		lbr_iiwa_14_R820_1.move(ptp(-2.19, 0.84, -0.47, -1.83, -1.11, -1.59,
				-1.78).setJointVelocityRel(0.2));
		getLogger().info("hand guiding active");
		lbr_iiwa_14_R820_1.setESMState("2");
		HandGuidingMotion motion = handGuiding();
		HandGuidingControlMode mode = new HandGuidingControlMode();
		mode.setMaxJointSpeed(100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0);
		motion.setMode(mode);
		DataRecorder rec = new DataRecorder();
		rec.setFileName("trajectory.txt");
		rec.setSampleInterval(50);
		rec.setTimeout(100, TimeUnit.SECONDS);
		rec.addCurrentJointPosition(lbr_iiwa_14_R820_1, AngleUnit.Radian);
		rec.enable();
		rec.startRecording();
		lbr_iiwa_14_R820_1.move(motion);
		rec.stopRecording();
		getLogger().info("hand guiding finished");
		lbr_iiwa_14_R820_1.setESMState("1");

		try {
			if (rec.awaitFileAvailable(5, TimeUnit.SECONDS)) {
				BufferedReader br;
				br = new BufferedReader(new FileReader(new File(rec.getURL()
						.getPath())));

				String line = br.readLine();
				ArrayList<RecordedJointState> joints = new ArrayList<RecordedJointState>();
				while ((line = br.readLine()) != null) {
					String[] numbers = line.split("\\s+");
					RecordedJointState js = new RecordedJointState();
					js.time = Double.valueOf(numbers[1])
							+ Double.valueOf(numbers[2]) / 1000000.0;
					js.pos = new JointPosition(Double.valueOf(numbers[3]),
							Double.valueOf(numbers[4]),
							Double.valueOf(numbers[5]),
							Double.valueOf(numbers[6]),
							Double.valueOf(numbers[7]),
							Double.valueOf(numbers[8]),
							Double.valueOf(numbers[9]));
					joints.add(js);
				}

				if (joints.size() == 0) {
					getLogger().info("No Joints - abort");
					return;
				}

				getLogger().info("Found " + joints.size() + " joints");

				while (true) {
					lbr_iiwa_14_R820_1.move(ptp(joints.get(0).pos)
							.setJointAccelerationRel(0.2).setJointVelocityRel(
									0.2));

					getApplicationControl().halt();

					SmartServo smart = new SmartServo(
							lbr_iiwa_14_R820_1.getCurrentJointPosition());
					// smart.setJointAccelerationRel(1.0);
					// smart.setSpeedTimeoutAfterGoalReach(0);
					// smart.setTimeoutAfterGoalReach(0.1);
					smart.setJointVelocityRel(0.1);
					lbr_iiwa_14_R820_1.moveAsync(smart);
					ISmartServoRuntime runtime = smart.getRuntime();
					// runtime.activateVelocityPlanning(true);

					for (int i = 1; i < joints.size(); ++i) {
						runtime.setDestination(joints.get(i).pos);

						while (runtime.getRemainingTime() > 0.15) {
							runtime.updateWithRealtimeSystem();
							ThreadUtil.milliSleep(1);
						}
					}
					
					runtime.stopMotion();
					getApplicationControl().halt();
				}

			} else {
				getLogger().info("No Log file");
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		HandGuiding app = new HandGuiding();
		app.getLogger().warn("called");
		app.runApplication();
	}

}
