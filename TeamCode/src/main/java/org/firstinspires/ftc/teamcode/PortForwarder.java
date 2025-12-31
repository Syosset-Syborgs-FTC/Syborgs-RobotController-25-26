package org.firstinspires.ftc.teamcode;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class PortForwarder {
	// STATIC reference to ensure only one manager runs at a time (prevents "Address in use")
	private static PortForwarder activeInstance = null;

	private final String targetIp;
	private final int[] ports;

	// Thread pool shared across all ports
	private ExecutorService executor;

	// Track all open server sockets so we can close them all on stop
	private final List<ServerSocket> activeSockets = Collections.synchronizedList(new ArrayList<>());

	private volatile boolean isRunning = false;

	/**
	 * Create a new PortForwarder for multiple ports.
	 * @param targetIp The IP of the device (e.g., "172.29.0.1")
	 * @param ports List of ports to forward (Local Port == Target Port)
	 */
	public PortForwarder(String targetIp, int... ports) {
		this.targetIp = targetIp;
		this.ports = ports;
	}

	/**
	 * Starts forwarding all configured ports.
	 * If another instance is running, it stops it first.
	 */
	public synchronized void start() {
		// 1. Safety Check: Kill any old instance from a previous OpMode run
		if (activeInstance != null) {
			activeInstance.stop();
		}
		activeInstance = this;

		if (isRunning) return;
		isRunning = true;

		// Create a cached thread pool to handle multiple connections across multiple ports
		executor = Executors.newCachedThreadPool();

		// 2. Launch a listener thread for EACH port in the array
		for (int port : ports) {
			executor.submit(() -> runServer(port));
		}
	}

	/**
	 * Stops all listeners and closes all connections.
	 */
	public synchronized void stop() {
		if (!isRunning) return;
		isRunning = false;

		// Clear static reference if we are the one holding it
		if (activeInstance == this) {
			activeInstance = null;
		}

		// Close all ServerSockets to stop listening
		synchronized (activeSockets) {
			for (ServerSocket socket : activeSockets) {
				try {
					if (socket != null && !socket.isClosed()) {
						socket.close();
					}
				} catch (IOException e) {
					// Ignore close errors
				}
			}
			activeSockets.clear();
		}

		// Shut down the thread pool
		if (executor != null) {
			executor.shutdownNow();
			try {
				executor.awaitTermination(200, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			}
		}
	}

	/**
	 * Internal logic to listen on a specific port
	 */
	private void runServer(int port) {
		try {
			ServerSocket serverSocket = new ServerSocket(port);
			activeSockets.add(serverSocket); // Track it so we can close it later

			while (isRunning && !serverSocket.isClosed()) {
				try {
					// Block waiting for a connection (e.g., from the Laptop)
					Socket clientSocket = serverSocket.accept();

					if (isRunning) {
						// Handle the data transfer in a separate thread
						executor.submit(() -> handleClient(clientSocket, port));
					} else {
						clientSocket.close();
					}
				} catch (IOException e) {
					// Socket closed or error, loop will check isRunning
				}
			}
		} catch (IOException e) {
			e.printStackTrace(); // Log error if port is permanently busy
		}
	}

	/**
	 * Bridges the connection between Client (Laptop) and Target (Limelight)
	 */
	private void handleClient(Socket clientSocket, int targetPort) {
		Socket targetSocket = null;
		try {
			// Connect to the internal device (172.29.0.1) on the same port
			targetSocket = new Socket(targetIp, targetPort);

			final InputStream clientIn = clientSocket.getInputStream();
			final OutputStream clientOut = clientSocket.getOutputStream();
			final InputStream targetIn = targetSocket.getInputStream();
			final OutputStream targetOut = targetSocket.getOutputStream();

			// Stream A: Laptop -> Limelight
			executor.submit(() -> copyStream(clientIn, targetOut));

			// Stream B: Limelight -> Laptop (run in current thread)
			copyStream(targetIn, clientOut);

		} catch (IOException e) {
			// Connection failed
		} finally {
			closeQuietly(clientSocket);
			closeQuietly(targetSocket);
		}
	}

	private void copyStream(InputStream input, OutputStream output) {
		byte[] buffer = new byte[4096];
		int bytesRead;
		try {
			while (isRunning && (bytesRead = input.read(buffer)) != -1) {
				output.write(buffer, 0, bytesRead);
				output.flush();
			}
		} catch (IOException e) {
			// Stream closed
		}
	}

	private void closeQuietly(Socket socket) {
		if (socket != null) {
			try {
				socket.close();
			} catch (IOException e) { /* ignore */ }
		}
	}
}