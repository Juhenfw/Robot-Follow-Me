import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rplidar import RPLidar
import tkinter as tk
from threading import Thread
import time
import os

# LIDAR Configuration with fallback options
PRIMARY_PORT = "/dev/robot_lidar"
FALLBACK_PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyAMA0"]
PRIMARY_BAUDRATE = 256000
FALLBACK_BAUDRATE = 115200

# Global variables
angles = []
distances = []
colors = []
running = False
thread = None
lidar = None
status_text = None
connected = False

def find_lidar_port():
    """Find an available LIDAR port"""
    if os.path.exists(PRIMARY_PORT):
        return PRIMARY_PORT
    
    print(f"Warning: {PRIMARY_PORT} not found. Looking for alternatives...")
    for port in FALLBACK_PORTS:
        if os.path.exists(port):
            print(f"Found alternative port: {port}")
            return port
    
    return None

def connect_lidar():
    """Connect to LIDAR with fallback options"""
    global lidar, status_text, connected
    
    port = find_lidar_port()
    if not port:
        update_status("Error: No LIDAR port found")
        return False
    
    # Try primary baudrate first
    try:
        update_status(f"Connecting to LIDAR on {port} at {PRIMARY_BAUDRATE}...")
        lidar = RPLidar(port, baudrate=PRIMARY_BAUDRATE)
        lidar.get_info()  # Test if connection works
        update_status(f"Connected to LIDAR on {port}")
        connected = True
        return True
    except Exception as e:
        print(f"Error with primary settings: {e}")
        
        # Try with fallback baudrate
        try:
            if lidar:
                try:
                    lidar.disconnect()
                except:
                    pass
            
            update_status(f"Trying fallback baudrate {FALLBACK_BAUDRATE}...")
            lidar = RPLidar(port, baudrate=FALLBACK_BAUDRATE)
            lidar.get_info()  # Test if connection works
            update_status(f"Connected to LIDAR on {port} with fallback settings")
            connected = True
            return True
        except Exception as e2:
            update_status(f"LIDAR connection failed: {e2}")
            return False

def update_status(message):
    """Update status message in GUI"""
    global status_text
    if status_text:
        status_text.config(text=message)
    print(message)

def update_lidar():
    """Thread function to continuously update LIDAR data"""
    global angles, distances, colors, running, lidar, connected
    
    if not connected:
        if not connect_lidar():
            update_status("Failed to connect to LIDAR")
            return
    
    try:
        update_status("Scanning...")
        for scan in lidar.iter_scans():
            if not running:
                break  # Exit loop if LIDAR is stopped
            
            # Clear previous data
            angles.clear()
            distances.clear()
            colors.clear()
            
            # Process scan data
            for _, angle, distance in scan:
                if distance <= 2000:  # Limit to distances ≤ 2000 mm
                    angles.append(np.deg2rad(angle))  # Convert to radians
                    distances.append(distance)
                    
                    # Determine color based on distance
                    if distance < 500:
                        colors.append('red')  # Close objects
                    elif distance < 1500:
                        colors.append('orange')  # Medium distance
                    else:
                        colors.append('purple')  # Far objects
        
    except Exception as e:
        update_status(f"Error during scan: {e}")
    finally:
        try:
            if lidar:
                lidar.stop()
        except:
            pass

def animate(i):
    """Animation function for matplotlib"""
    plt.cla()
    plt.title("LiDAR Real-Time Scan")
    ax = plt.gca()
    
    # Set fixed limits for consistent visualization
    ax.set_rlim(0, 2000)
    
    # Add distance rings
    circles = [500, 1000, 1500, 2000]
    for circle in circles:
        ax.text(0, circle, f"{circle}mm", ha='center', va='bottom', alpha=0.7, size=8)
    
    # Plot points
    if angles and distances:
        for angle, distance, color in zip(angles, distances, colors):
            plt.polar(angle, distance, 'o', color=color, markersize=3)
    
    return []

def start_lidar():
    """Start LIDAR scanning"""
    global running, thread, connected
    
    if not running:
        running = True
        thread = Thread(target=update_lidar, daemon=True)
        thread.start()
        update_status("LIDAR started")

def stop_lidar():
    """Stop LIDAR scanning"""
    global running
    
    if running:
        update_status("Stopping LIDAR...")
        running = False
        
        # Wait for thread to complete
        if thread and thread.is_alive():
            thread.join(timeout=2)
        
        update_status("LIDAR stopped")

def close_program():
    """Safely close the program"""
    global running, lidar
    
    update_status("Closing program...")
    running = False
    
    # Stop and disconnect LIDAR
    if lidar:
        try:
            lidar.stop_motor()
        except:
            pass
        
        try:
            lidar.stop()
            lidar.disconnect()
        except:
            pass
    
    time.sleep(0.5)  # Wait for threads
    plt.close('all')  # Close matplotlib
    root.quit()
    root.destroy()

# Create GUI
root = tk.Tk()
root.title("LiDAR Scanner")
root.protocol("WM_DELETE_WINDOW", close_program)  # Handle window close

# Main frame
main_frame = tk.Frame(root)
main_frame.pack(padx=10, pady=10)

# Status display
status_text = tk.Label(main_frame, text="Ready to connect", width=40, anchor="w")
status_text.pack(pady=(0, 10))

# Control buttons
button_frame = tk.Frame(main_frame)
button_frame.pack()

btn_start = tk.Button(button_frame, text="Start LiDAR", command=start_lidar, width=15)
btn_start.pack(side=tk.LEFT, padx=5)

btn_stop = tk.Button(button_frame, text="Stop LiDAR", command=stop_lidar, width=15)
btn_stop.pack(side=tk.LEFT, padx=5)

btn_quit = tk.Button(button_frame, text="Exit", command=close_program, width=15)
btn_quit.pack(side=tk.LEFT, padx=5)

# Initialize Matplotlib
fig = plt.figure(figsize=(8, 8))
ax = plt.subplot(111, projection='polar')
ani = animation.FuncAnimation(fig, animate, interval=100)

# Display GUI
plt.show(block=False)  # Prevents Matplotlib from blocking Tkinter
root.mainloop()
