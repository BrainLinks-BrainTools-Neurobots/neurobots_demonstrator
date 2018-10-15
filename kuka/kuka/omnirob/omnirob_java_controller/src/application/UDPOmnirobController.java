package application;

import com.kuka.common.ThreadUtil;
import java.util.HashMap; 
import java.io.*;

public class UDPOmnirobController {
	private UDPPlatformController platformController;
	private UDPLBRController lbrController;
	private UDPLBRJointStatePublisher lbrJointStatePublisher;
	private HashMap<String, Integer> portMap;

	public void initialize(double platformAccelX, double platformAccelY, 
			double platformAccelTheta, double jointVelo, String path)  throws IOException {
		// Create controller
		
		platformController = new UDPPlatformController();
		lbrController = new UDPLBRController();
		lbrJointStatePublisher = new UDPLBRJointStatePublisher();
		init(platformAccelX, platformAccelY, platformAccelTheta, jointVelo, path);
	}

	public void init(double platformAccelX, double platformAccelY, double platformAccelTheta,
			double jointVelo, String path) throws IOException {
		// Give each controller one port
		String fullpath = path + "/../../kuka_manager/config.txt";
		FileReader fr = new FileReader(new File(fullpath));
		BufferedReader br = new BufferedReader(fr);
		String line = br.readLine();
		portMap = new HashMap<String, Integer>();
		while(line != null) {
			String[] vars = line.split(" ");
			String name = vars[0];
			int port = Integer.parseInt(vars[1]);
			portMap.put(name, port);
			line = br.readLine();
		}

		lbrController.init(portMap.get("OmnirobControllerPort"), jointVelo, 1, path);
		platformController.init(portMap.get("OmnirobPlatformPublisherPort"), 
				platformAccelX, platformAccelY, platformAccelTheta, path);
		lbrJointStatePublisher.init(portMap.get("OmnirobPublisherPort"),
				portMap.get("OmnirobReceiverPort"), path);
	}

	public void run() {
		// Start threads
		
		platformController.start();
		lbrJointStatePublisher.start();

		while (true) {
			ThreadUtil.nanoSleep(100);
		}
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 5) {
			System.out.println("Parameters needed: [a_x] [a_y] [a_theta] [joint_velocity] [path]");
			return;
		}
		UDPOmnirobController controller = new UDPOmnirobController();
		controller.initialize(Double.parseDouble(args[0]), Double.parseDouble(args[1]), Double.parseDouble(args[2]),
				Double.parseDouble(args[3]), args[4]);
		controller.run();
	}
}
