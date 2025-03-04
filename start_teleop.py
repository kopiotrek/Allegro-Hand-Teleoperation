import tkinter as tk
import os
import signal
import subprocess
import psutil
import time 

# Paths
SCRIPTS_DIR = os.path.expanduser("~/RPL/Allegro-Hand-Teleoperation/ik_teleop/")
ALLEGRO_HAND_DIR = os.path.expanduser("~/RPL/DIME-Controllers/")
ACTIVATE_ENV = "source /home/piotr/RPL/Allegro-Hand-Teleoperation/ik_teleop/env_teleop/bin/activate && source ~/RPL/devel/setup.bash"

# List of scripts
scripts = {
    "Allegro Hand": f"source {ALLEGRO_HAND_DIR}/devel/setup.bash && roslaunch allegro_hand allegro_hand.launch",
    "TCP Endpoint": "roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=192.168.7.108 tcp_port:=10000",
    "Allegro Controller": f"python {SCRIPTS_DIR}/allegro_controller.py",
    "Index Controller": f"python {SCRIPTS_DIR}/index_controller.py",
    "Middle Controller": f"python {SCRIPTS_DIR}/middle_controller.py",
    "Ring Controller": f"python {SCRIPTS_DIR}/ring_controller.py",
    "Thumb Controller": f"python {SCRIPTS_DIR}/thumb_controller.py",
    "Hand Alignment": f"python {SCRIPTS_DIR}/ik_core/hands_aligment.py",
}

processes = {}  # Dictionary to store running processes

def start_script(script_name):
    """Start a script as a background process."""
    if script_name in processes and processes[script_name].poll() is None:
        return  # If already running, do nothing
    
    cmd = f'bash -c "{ACTIVATE_ENV}; {scripts[script_name]}"'
    process = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
    processes[script_name] = process
    update_buttons()

def stop_script(script_name):
    """Stop a script if it's running."""
    if script_name in processes and processes[script_name].poll() is None:
        process = processes[script_name]
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # Terminate process group
        process.wait()
        del processes[script_name]
    update_buttons()

def stop_all():
    """Stop all running scripts."""
    for script_name in list(processes.keys()):
        stop_script(script_name)

def start_all():
    """Start all scripts."""
    for script_name in scripts:
        start_script(script_name)
        time.sleep(0.5)

def is_running(script_name):
    """Check if a process is running."""
    return script_name in processes and processes[script_name].poll() is None

def update_buttons():
    """Update button colors based on script status."""
    for script_name, button in script_buttons.items():
        button.config(bg="green" if is_running(script_name) else "gray")

def on_closing():
    """Handle application exit."""
    stop_all()
    root.destroy()

# GUI Setup
root = tk.Tk()
root.title("Teleoperation Control Panel")

script_buttons = {}

for script_name in scripts:
    btn = tk.Button(root, text=f"Start {script_name}", command=lambda s=script_name: start_script(s), bg="gray")
    btn.pack(fill=tk.X)
    script_buttons[script_name] = btn

tk.Button(root, text="Start All", command=start_all, fg="white", bg="blue").pack(fill=tk.X)
tk.Button(root, text="Stop All", command=stop_all, fg="white", bg="red").pack(fill=tk.X)

# Add signal handler for Ctrl+C
root.protocol("WM_DELETE_WINDOW", on_closing)
signal.signal(signal.SIGINT, lambda sig, frame: on_closing())

root.mainloop()
