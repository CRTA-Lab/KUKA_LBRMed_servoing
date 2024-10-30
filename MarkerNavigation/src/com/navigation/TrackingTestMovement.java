package com.navigation;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.med.deviceModel.LBRMed;

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
public class TrackingTestMovement extends RoboticsAPIApplication {
	@Inject
	private LBRMed robot;
	private Tool pointer;
	private ObjectFrame marker;
	
	private ObjectFrame P1;
	private ObjectFrame P2;
	private ObjectFrame P3;
	private ObjectFrame P4;
	
	private BufferedReader in;
	private PrintWriter out;
	private ServerSocket server;
	private Socket client;
	private String msg;
	
	

	@Override
	public void initialize() {
		pointer = getApplicationData().createFromTemplate("pointer_w_marker"); // stvaranje alata iz tool templatea
		pointer.attachTo(robot.getFlange()); // spajanje alta na prihvatnicu robota
		marker = getApplicationData().getFrame("/fixed_marker"); // ucitavanje fiksnog markera
		
		try {
			// stvaranje TCP/IP sserver socketa i stvaranje ulaznog i izlaznog streama podataka
			
			server = new ServerSocket(30000);
			client = server.accept();
			
			
			in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		    out = new PrintWriter(client.getOutputStream(), true);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		
		
	}

	@Override
	public void run() {
		getApplicationControl().setApplicationOverride(0.3);
		
		P1 = marker.getChild("P1");
		P2 = marker.getChild("P2");
		P3 = marker.getChild("P3");
		P4 = marker.getChild("P4");
		
		pointer.getFrame("TCP").move(ptp(P1).setBlendingRel(0));
		
		out.println("Starting static");
		
		try {
			TimeUnit.SECONDS.sleep(13);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		out.println("Starting movement");

		try {
			TimeUnit.MILLISECONDS.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		pointer.getFrame("TCP").move(ptp(P2).setBlendingRel(0));
		pointer.getFrame("TCP").move(ptp(P3).setBlendingRel(0));
		pointer.getFrame("TCP").move(ptp(P4).setBlendingRel(0));
		pointer.getFrame("TCP").move(ptp(P1).setBlendingRel(0));
		
		out.println("Done");
		
	}
	@Override
	public void dispose(){
		try {
			server.close();
		
			client.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}