package application;

import java.io.File;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// Kuka stuff
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.controllerModel.sunrise.ParameterConfiguration;
import com.kuka.roboticsAPI.RoboticsAPIContext;
import com.kuka.roboticsAPI.OmniRob.OmniRobPlatformUtilities;
import com.kuka.roboticsAPI.OmniRob.motionModel.IRealtimeMoveRuntime;
import com.kuka.roboticsAPI.OmniRob.motionModel.OmniRobRealtimeMove;
import com.kuka.roboticsAPI.omniMove.OmniRobPlatform;

//Thread stuff
import java.lang.Thread;

public class UDPPlatformController extends Thread {
	private PlatformDaemon platform;
   
	public class PlatformDaemon {
		private OmniRobPlatform theOmniRob;
		private int port;
		double xAccel = 5;
		double yAccel = 5;
		double thetaAccel = 1;
		private String path;

		public PlatformDaemon(String path) {
			this.path = path;
		}

		public void initialize() {
			RoboticsAPIContext context;
			File configFile = new File(path + "/RoboticsAPI.config.xml");
			context = RoboticsAPIContext.createFromFile(configFile);
			theOmniRob = OmniRobPlatformUtilities.locateOmniRob(context);
		}

		public void initNetwork(int port) {
			this.port = port;
		}

		public void initSpeedCommands(double xAccel, double yAccel, double thetaAccel) {
			this.xAccel = xAccel;
			this.yAccel = yAccel;
			this.thetaAccel = thetaAccel;
		}

		public void run() {
			DatagramSocket serverSocket;
			try {
				// Setup omniRob stuff
				OmniRobRealtimeMove cartSpeedCommander = new OmniRobRealtimeMove();
				cartSpeedCommander.setCollisionAvoidanceParameterApproachDist(0.0);
				cartSpeedCommander.setCollisionAvoidanceParameterSideDist(0.0);
				cartSpeedCommander.setMaxAcceleration(xAccel, yAccel, thetaAccel);

				cartSpeedCommander.setCommandErrTimeout(-1);
				cartSpeedCommander.setActiveSpeedTimeout(1);
				IRealtimeMoveRuntime realtimeMoveRuntime = OmniRobPlatformUtilities.startRealtimeMove(theOmniRob, cartSpeedCommander);
				ThreadUtil.milliSleep(100);
				realtimeMoveRuntime.setActiveSpeedTimeout(1);

				// Set colli parameter
				//ParameterConfiguration.Current.addParameter("CollisionAvoidance", "rot_acceleration", 100.0);
				//ParameterConfiguration.Current.addParameter("CollisionAvoidance", "acceleration", 100.0);
				//ParameterConfiguration.Current.addParameter("CollisionAvoidance", "deceleration", 100);
				//ParameterConfiguration.Current.addParameter("CollisionAvoidance", "reaction_time", 0.0000001);
				// ParameterConfiguration.Current.addParameter("CollisionAvoidance", "use_laser", 0);
				/*ParameterConfiguration.Current.addParameter("CollisionAvoidance", "approach_dist", 0.0);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "side_dist ", 0.075);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "length", 1.196);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "width", 0.721);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "odometry_center_offset", 0.0);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "deceleration", xAccel / 2.0);
				ParameterConfiguration.Current.addParameter("CollisionAvoidance", "rot_acceleration", yAccel / 2.0);*/

				serverSocket = new DatagramSocket(port);
				byte[] receiveData = new byte[1024];
				System.out.println("Waiting for platform commands");
				while (true) {
					try {
						DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
						serverSocket.receive(receivePacket);
						ByteBuffer bf = ByteBuffer.wrap(receivePacket.getData());
						bf.order(ByteOrder.LITTLE_ENDIAN);

						int counter = bf.getInt(0);
						short mode = bf.getShort(4);
						if (mode == 0) {
							double v_x = bf.getDouble(6);
							double v_y = bf.getDouble(14);
							double v_theta = bf.getDouble(22);
							long tstamp = bf.getLong(30);

							// Create message
							// String header = "New Message of speed cmd:\n" +
							// tstamp + "\n" + counter + "\n";
							// String odometry = "" + v_x + "\n" + v_y + "\n" +
							// v_theta + "\n";

							// System.out.println(header + odometry);

							// command to drive
							realtimeMoveRuntime.setRealtimeDestination(v_x, v_y, v_theta);
						} else if (mode == 3) {
							double accX = bf.getDouble(38);
							double accY = bf.getDouble(46);
							double accTheta = bf.getDouble(62);

							xAccel = accX;
							yAccel = accY;
							thetaAccel = accTheta;

							// Create message
							String header = "New Message of speeds:\n" + 0 + "\n" + counter + "\n";
							String odometry = "" + accX + "\n" + accY + "\n" + accTheta + "\n";

							System.out.println(header + odometry);

							cartSpeedCommander.setMaxAcceleration(xAccel, yAccel, thetaAccel);
							realtimeMoveRuntime.stopMotion();
							realtimeMoveRuntime = OmniRobPlatformUtilities.startRealtimeMove(theOmniRob, cartSpeedCommander);
						}
					} catch (IllegalStateException e) {
						System.out.println("Illegal State!");
						e.printStackTrace();
						realtimeMoveRuntime.stopMotion();
						realtimeMoveRuntime = OmniRobPlatformUtilities.startRealtimeMove(theOmniRob, cartSpeedCommander);
					}
				}
			} catch (SocketException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();

			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public void init(int port, double xAccel, double yAccel, double thetaAccel, String path) {
		platform = new PlatformDaemon(path);
		platform.initialize();
		platform.initNetwork(port);
		platform.initSpeedCommands(xAccel, yAccel, thetaAccel);
	}

	@Override
	public void run() {
		// Let the application run
		// platform.runApplication();
		platform.run();
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 4) {
			System.out.println("Parameters needed: [port] [a_x] [a_y] [a_theta] [path]");
			return;
		}
		UDPPlatformController controller = new UDPPlatformController();
		controller.init(Integer.parseInt(args[0]), Double.parseDouble(args[1]), Double.parseDouble(args[2]), Double.parseDouble(args[3]), args[4]);
		controller.start();
	}
}
