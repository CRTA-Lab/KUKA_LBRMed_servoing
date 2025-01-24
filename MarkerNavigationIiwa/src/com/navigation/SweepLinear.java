package com.navigation;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

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
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class SweepLinear extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	
	private Tool sonda;
	private ObjectFrame marker;
	private Frame target;
	private Frame ref_frame;
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;

	@Override
	public void initialize() {
		sonda = getApplicationData().createFromTemplate("SonoScape_sonda");
		sonda.attachTo(robot.getFlange());
		
		
		marker = getApplicationData().getFrame("/zero_frame");
		ref_frame = marker.copy();
		ref_frame.setParent(sonda.getFrame("os_sonde"), false);
		ref_frame.setParent(marker, true);
		
		
		target = marker.getChild("sweep_frame_lin").copy();
		target.setParent(sonda.getFrame("os_sonde"));
		
		
		try {
			server = new ServerSocket(30001);
		
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		getLogger().info("Init done");
		
	}

	@Override
	public void run() {
		target = marker.getChild("sweep_frame_lin").copy();
		target.setParent(sonda.getFrame("os_sonde"));
		target.setParent(marker, true);
		while (true){
			sonda.getFrame("os_sonde").move(lin(target).setJointVelocityRel(0.15).setBlendingOri(0.05).setJointJerkRel(0.3));
			out.println(sonda.getFrame("os_sonde").transformationTo(ref_frame).toString());
			getLogger().info(sonda.getFrame("os_sonde").transformationTo(ref_frame).toString());
			try {
				msg = in.readLine();
			} catch (IOException e) {
				e.printStackTrace();
				break;
			}
			target = marker.getChild("sweep_frame_lin").copy();
			target.setParent(sonda.getFrame("os_sonde"));
			target.setParent(marker, true);
			
		}
	}
	
	
	public void dispose(){
		try {
			server.close();
		
			client.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}