package application;

import java.awt.Button;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class Quit extends Frame {
	public Quit(String appName) {
		setTitle(appName);
		// default layout manager for "Frame" is "BorderLayout"

		Button q = new Button("Quit " + appName);

		class QuitButtonListener implements ActionListener {
			public void actionPerformed(ActionEvent e) {
				close();
			}
		}
		q.addActionListener(new QuitButtonListener());
		add("Center", q);

		class WindowClosingListener extends WindowAdapter {
			public void windowClosing(WindowEvent evt) {
				close();
			}
		}
		addWindowListener(new WindowClosingListener());
		setSize(150, 100);
		setVisible(true);
	}

	void close() {
		setVisible(false);
		dispose();
		System.exit(0);
	}
}
