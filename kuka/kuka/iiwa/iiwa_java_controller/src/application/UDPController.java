package application;

import com.kuka.common.ThreadUtil;
import java.util.HashMap;
import java.io.*;

public class UDPController {
	private UDPIIWAController iiwaController;
	private UDPIIWAJointStatePublisher iiwaJointStatePublisher;
	private HashMap<String, Integer> portMap;

	public void initialize(double jointVelo, String path) throws IOException {
		iiwaController = new UDPIIWAController();
		iiwaJointStatePublisher = new UDPIIWAJointStatePublisher();
		init(jointVelo, path);
	}

	public void init(double jointVelo, String path) throws IOException {
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
		
		iiwaController.init(portMap.get("IIWAControllerPort"), jointVelo, 1, path);
		iiwaJointStatePublisher.init(portMap.get("IIWAPublisherPort"),
				portMap.get("IIWAReceiverPort"), path);
	}

	public void run() {
		iiwaJointStatePublisher.start();

		while (true) {
			ThreadUtil.nanoSleep(100);
		}
	}

	public static void main(String args[]) throws Exception {
		if (args.length != 2) {
			System.out.println("Parameters needed: [joint_velocity] [path]");
			return;
		}
		UDPController controller = new UDPController();
		controller.initialize(Double.parseDouble(args[0]), args[1]);
		controller.run();
	}
}
