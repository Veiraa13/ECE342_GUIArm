import math
import tkinter as tk
from tkinter import scrolledtext, ttk, messagebox
import serial
import time
import serial.tools.list_ports

# Declaring variables
L1 = 200.0 
L2 = 200.0
squareSides = 10.0
homeAngle1 = 45.0
homeAngle2 = 45.0

def computeOffset(homeAngle1, homeAngle2):
    """
    Finds the (x, y) position of the tip when joints sit at the given angles
    """

    t1 = math.radians(homeAngle1)
    t2 = math.radians(homeAngle2)
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return [x, y]

def _now():
    return time.strftime("%H:%M:%S")

class consolePane(scrolledtext.ScrolledText):
    """ 
    Color coding for console pane. 
    """
    def __init__(self, master, width = 54, height = 30, **kwargs):
        super().__init__(master, width = width, height = height, state = tk.DISABLED, 
                         bg = "#1e1e1e", fg = "#e0e0e0", insertbackground = "#e0e0e0", 
                         font = ("Consolas", 9), **kwargs)
        
        # Identifying colors for console pane display text
        colors = {
            "SEND": "#3498db",
            "INFO": "#2ecc71",
            "ERROR": "#e74c3c"
        }

        for tag, color in colors.items():
            self.tag_config(tag, foreground = color)
        
    def append(self, message, tag):
        """ 
        Enable, insert colored message, disable, and scroll to bottom. 
        """
        self.config(state = tk.NORMAL)
        self.insert(tk.END, message + "\n", tag)
        self.config(state = tk.DISABLED)
        self.yview_moveto(1.0)

    def send(self, msg):
        """ 
        Logs outgoing messages in blue. 
        """
        self.append(msg, "SEND")

    def info(self, msg):
        """ 
        Logs informational messages in green. 
        """
        self.append(msg, "INFO")

    def error(self, msg):
        """ 
        Logs error messages in red. 
        """
        self.append(msg, "ERROR")

class inverseKinematics:
    """ 
    Class responsible for inverse kinematics, finding shoulder and elbow angles for desired coordinates in mm. 
    """
    
    def __init__(self, L1 = 200.0, L2 = 200.0, zero_offset1 = 0.0, zero_offset2 = 0.0, homeAngle1 = 45.0, homeAngle2 = 45.0):
        self.L1 = L1
        self.L2 = L2
        self.zero_offset1 = zero_offset1
        self.zero_offset2 = zero_offset2

    def solve(self, x, y):
        """
        Standard 2 pivot arm invers kinematices
        """
        # Calculate distance from origin to target
        d = math.hypot(x, y)
        
        # Check reachability
        if d > (self.L1 + self.L2):
            raise ValueError("Target location is out of reach")
        if d < abs(self.L1 - self.L2):
            raise ValueError("Target location is too close")
        
        # Handle special case: target at origin
        if d < 1e-9:
            return (90.0 + self.zero_offset1, 90.0 + self.zero_offset2)
        
        # Standard IK solution
        # Calculate elbow angle first (easier)
        cos_elbow = (self.L1**2 + self.L2**2 - d**2) / (2 * self.L1 * self.L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        
        # This gives us the interior angle between the two links
        elbow_interior_angle = math.acos(cos_elbow)
        
        # Calculate shoulder angle
        angle_to_target = math.atan2(y, x)
        cos_alpha = (self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)
        
        shoulder_angle = angle_to_target - alpha
        
        # Calculate where the elbow joint is, then find angle to target
        elbow_x = self.L1 * math.cos(shoulder_angle)
        elbow_y = self.L1 * math.sin(shoulder_angle)
        
        # Angle from elbow joint to target
        elbow_angle = math.atan2(y - elbow_y, x - elbow_x)
        
        # Convert to degrees and apply offsets
        shoulder_deg = math.degrees(shoulder_angle) + self.zero_offset1
        elbow_deg = math.degrees(elbow_angle) + self.zero_offset2
        
        # Convert elbow angle from standard math convention to servo convention
        elbow_deg = 180 - elbow_deg
        if elbow_deg < 0:
            elbow_deg += 360
        elif elbow_deg > 360:
            elbow_deg -= 360
        
        # Apply servo constraints
        shoulder_deg = max(0, min(180, shoulder_deg))
        elbow_deg = max(0, min(180, elbow_deg))
        
        return shoulder_deg, elbow_deg

class serialHandler:
    """ 
    Handles serial communication between computer and ESP32. 
    """

    def __init__(self, baudrate: int = 9600, timeout: float = 2.0):
        """
        Opening serial connection to ESP32.
        """
        # Finds the port and connects
        port = self.findEsp32Port()
        if port is None:
            raise RuntimeError("ESP32 not found.")
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        
        # Allow time for ESP32 to connect
        time.sleep(2.0)

    def findEsp32Port(self) -> str | None:
        """
        Finding the port the ESP32 is connected to
        """
        for port in serial.tools.list_ports.comports():
            desc = port.description.lower()
            if "silicon labs" in desc or "cp210" in desc:
                return port.device
        return None
    
    def sendAngles(self, shoulderAngle: float, elbowAngle: float) -> str:
        """ 
        Formats and sends packet containing servo angles in the form of XXXYYY 
        where X is the ceiling of the shoulder angle and Y is the ceiling of the elbow angle.
        """
        # Ceiling of the shoulder and elbow angles (degrees)
        shoulder = math.ceil(shoulderAngle)
        elbow = math.ceil(elbowAngle)

        # Creates the packet to be sent in the form of 'XXXYYY'
        packet = f"{shoulder:03d}{elbow:03d}"

        # Writes the packet to the ESP32
        self.ser.write(packet.encode())
        return packet

    def close(self) -> None:
        """
        Closes the serial connection to the ESP32
        """
        self.ser.close()
        
class GUI:
    """
    Handles the creation and processing of the GUI
    """

    def __init__(self):
        # Setting up the interface window
        self.root = tk.Tk()
        self.root.title("Robotic Arm GUI")
        self.root.geometry("1100x600")

        # Console Pane
        self.console = consolePane(self.root)
        self.console.pack(side = "right", fill = "both", padx = 6, pady = 6)

        # Attempt serial connetion
        self.console.append(f"{_now()} Searching for ESP32 port...", "INFO")
        self.attemptConnection()
        
        # IK  solver
        self.ik = inverseKinematics(L1, L2)

        # Precompute home offset in XY
        self.homeOffset = computeOffset(homeAngle1, homeAngle2)
        
        # State
        self.tipPosition = list(self.homeOffset)
        self.units = 'mm'
        self.scale = 1.0
        self.squareSides= 20

        # Build the GUI
        self.buildLayout()
        self.root.mainloop()

    def attemptConnection(self):
        """
        Attempts to connect GUI to ESP32
        """
        try:
            self.serial = serialHandler()
            self.console.append(f"{_now()} ESP32 connected successfully", "INFO")
            self.serial.sendAngles(homeAngle1, homeAngle1)
            self.tipPosition = list(self.homeOffset)
        except Exception as e:
            self.serial = None
            self.console.append(f"{_now()} ESP32 not found: {e}", "ERROR")
            self.root.after(1000, self.attemptConnection)

    def buildLayout(self):
        """
        Builds the layout for the GUI
        """
        # Main container
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand = True, padx = 6, pady = 6)

        # Left side of interface (mm, in, End buttons)
        left = ttk.Frame(main)
        left.pack(side = "left", fill = "y", padx = (0, 12))
        ttk.Button(left, text = "G21 - mm", command = lambda: self.setUnits("mm")).pack(fill = "x", pady = 2)
        ttk.Button(left, text = "G20 - in", command = lambda: self.setUnits("in")).pack(fill = "x", pady = 2)
        ttk.Button(left, text = "End", command = self.end).pack(fill = "x", pady = (10, 2))

        # Center of intercae (Absolute and relative moves)
        center = ttk.Frame(main)
        center.pack(side = "left", fill = "y")

        def makeMoveBlock(parent, title, row, command):
            ttk.Label(parent, text = title).grid(row = row, column = 0, columnspan = 3, pady = (0, 4), sticky = "w")
            x = ttk.Entry(parent, width = 8); x.grid(row = row + 1, column = 0, padx = 4)
            y = ttk.Entry(parent, width = 8); y.grid(row = row + 1, column = 1, padx = 4)
            ttk.Button(parent, text = "Run", command = lambda: command(x, y)).grid(row = row + 1, column = 2)
            return x, y
        
        self.absX, self.absY = makeMoveBlock(center, "Move Absolute (X, Y)", 0, self.moveAbsolute)
        self.relX, self.relY = makeMoveBlock(center, "Move Relative (dX, dY)", 3, self.moveRelative)

        # Bottom of interface values (Stop and square buttons)
        bottom = ttk.Frame(self.root)
        bottom.pack(side = "bottom", fill = "x", pady = 8)
        ttk.Button(bottom, text = "Stop", width = 12, command = self.emergencyStop).pack(side = "left", padx = 6)
        ttk.Button(bottom, text = "Draw Pattern", width = 12, command = self.drawPattern).pack(side = "left", padx = 6)

        # Right side of interace (Console pane)
        self.console.pack(side = "right", fill = "both", padx = 6, pady = 6)

    def setUnits(self, unit):
        """
        Changes units between mm and in.
        """
        self.units = unit
        self.scale = 1.0 if unit == 'mm' else 25.4
        self.console.append(f"Units set to {unit}", "INFO")

    def end(self):
        """
        End program - sends 000000 packet to ESP32 and exits application
        """
        if not self.serial:
            self.console.error("Cannot end program: ESP32 is not connected")
            return
        
        try:
            # Send angles 0, 0 using sendAngles method
            packet = self.serial.sendAngles(0, 0)
            self.console.send(f"Program end -> {packet}")
            
            # Wait a moment for command to be processed
            time.sleep(1)
            
            # Close serial connection
            self.serial.close()
            self.serial = None
            
            self.console.append("Program ended successfully", "INFO")
            
            # Exit the application
            self.root.quit()
            self.root.destroy()
            
        except Exception as e:
            self.console.error(f"End program failed: {e}")
            # Still try to close connection and exit on error
            try:
                if self.serial:
                    self.serial.close()
                    self.serial = None
                self.root.quit()
                self.root.destroy()
            except:
                pass


    def moveAbsolute(self, x, y):
        """
        Calls inverseKinematics to move absolute to given corrdinates.
        """
        try:
            tempX = float(x.get()) * self.scale
            tempY = float(y.get()) * self.scale
        except ValueError:
            messagebox.showerror("Invalid Input", "Enter numeric X and Y")
            return
        
        # targetX = self.homeOffset[0] + tempX
        # targetY = self.homeOffset[1] + tempY

        # Special case: if user types (0,0)
        if abs(tempX) < 1e-9 and abs(tempY) < 1e-9:
            targetX, targetY = self.homeOffset[0], self.homeOffset[1]
        else:
            targetX, targetY = tempX, tempY


        try:
            cmd1, cmd2 = self.ik.solve(targetX, targetY)
            packet = self.serial.sendAngles(cmd1, cmd2)
            self.tipPosition = [targetX, targetY]
            self.console.send(f"Move Absolute -> {packet}")
        except Exception as e:
            self.console.error(f"Move Absolute failed: {e}")


    def moveRelative(self, x, y):
        """
        Move relative by calculating final position and using absolute move logic
        """
        try:
            deltaX = float(x.get()) * self.scale
            deltaY = float(y.get()) * self.scale
            
            # Calculate final position
            finalX = self.tipPosition[0] + deltaX
            finalY = self.tipPosition[1] + deltaY
            
            # Do inverse kinematics
            cmd1, cmd2 = self.ik.solve(finalX, finalY)
            
            # Send to ESP32
            packet = self.serial.sendAngles(cmd1, cmd2)
            
            # Update position
            self.tipPosition = [finalX, finalY]
            
            # Log it
            self.console.send(f"Move Relative -> {packet}")
            
        except Exception as e:
            self.console.error(f"Move Relative failed: {e}")

    
    def drawPattern(self):
        """
        Draws a pattern by moving the robotic arm to specific points.
        First moves to (1, 1), then to the destination point.
        """
        if not self.serial:
            self.console.error("Cannot draw pattern: ESP32 is not connected")
            return
        
        self.console.append("Starting pattern sequence...", "INFO")
        
        try:
            # First move to (1, 1)
            self.console.append("Moving to initial position (1, 1)...", "INFO")
            targetX = 1.0 * self.scale
            targetY = 1.0 * self.scale
            cmd1, cmd2 = self.ik.solve(targetX, targetY)
            packet = self.serial.sendAngles(cmd1, cmd2)
            self.tipPosition[0] = targetX
            self.tipPosition[1] = targetY
            self.console.send(f"Initial move -> {packet}")
            
            # Wait for arm to reach position
            time.sleep(1.5)
            
            # Then move to destination (400, 0)
            self.console.append("Moving to destination (400, 0)...", "INFO")
            targetX = 400.0 * self.scale
            targetY = 0.0 * self.scale
            cmd1, cmd2 = self.ik.solve(targetX, targetY)
            packet = self.serial.sendAngles(cmd1, cmd2)
            self.tipPosition[0] = targetX
            self.tipPosition[1] = targetY
            self.console.send(f"Destination move -> {packet}")
            
            self.console.append("Pattern sequence completed!", "INFO")
            
        except Exception as e:
            self.console.error(f"Unable to draw pattern: {e}")
    

    def emergencyStop(self):
        """
        Sends a emergency stop to ESP32 (360360).
        """
        try:
            packet = self.serial.sendAngles(360, 360)
            self.console.error("Emergency stop activated")
        except Exception as e:
            self.console.error(f"Emergency stop faile: {e}")

# Entry point
if __name__ == '__main__':
    GUI()