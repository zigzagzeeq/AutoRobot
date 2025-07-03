# AutoRobot
Minimum Requirements:

💻 Computer: Intel i5 (8th gen) or AMD Ryzen 5 equivalent
🧠 Memory: 8GB RAM (16GB recommended)
💾 Storage: 50GB free space
📷 Camera: USB webcam with 720p resolution or higher
🎤 Microphone: USB microphone or built-in laptop microphone
🔊 Speakers: External speakers or headphones
🌐 Internet: Stable connection (10 Mbps recommended)

Recommended Hardware:

📷 Camera: Logitech C920 or similar 1080p webcam
🎤 Microphone: Blue Yeti or similar USB microphone
🔊 Speakers: Dedicated desktop speakers for clear audio
💻 Graphics: NVIDIA GTX 1650 or better (for faster processing)

Software Requirements
Operating System:

🐧 Ubuntu 20.04 LTS (Primary support)
🪟 Windows 11 (Limited support via WSL2)
🍎 macOS 12+ (Experimental support)

Required Software:

ROS Noetic (Robot Operating System)
Python 3.8+
OpenCV 4.5+
TensorFlow 2.8+

Internet Services
Required Services:

Google Speech Recognition API (for speech-to-text)
Google Text-to-Speech API (for robot voice)
Optional: OpenAI API (for advanced conversations)

 Step-by-Step Installation
Step 1: Prepare Your System
bash# Update your system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y curl wget git python3-pip

# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y ros-noetic-desktop-full
Step 2: Download the Robot Software
bash# Create workspace directory
mkdir -p ~/emotional_robot_ws/src
cd ~/emotional_robot_ws/src

# Clone the repository
git clone https://github.com/emotional-robot/emotional-robot-assistant.git

# Navigate to workspace
cd ~/emotional_robot_ws

# Build the project
catkin_make
Step 3: Install Dependencies
bash# Install Python dependencies
pip3 install -r src/emotional-robot-assistant/requirements.txt

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y
Step 4: Configure Environment
bash# Add to your shell configuration
echo "source ~/emotional_robot_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_WORKSPACE=~/emotional_robot_ws" >> ~/.bashrc

# Reload configuration
source ~/.bashrc
Step 5: Download AI Models
bash# Download emotion detection model
cd ~/emotional_robot_ws/src/emotional-robot-assistant/models/
wget https://github.com/emotional-robot/models/releases/download/v1.0/emotion_model.h5

# Verify download
ls -la *.h5
3.2 Hardware Setup
Camera Setup

Connect your webcam to a USB 3.0 port
Test camera with: cheese or vlc v4l2:///dev/video0
Adjust positioning: Place camera at eye level, 2-3 feet away
Check lighting: Ensure good, even lighting on your face

Microphone Setup

Connect microphone to USB port
Test audio input: arecord -l to list devices
Test recording: arecord -d 5 test.wav then aplay test.wav
Adjust levels: Use alsamixer to set appropriate input levels

Speaker Setup

Connect speakers or headphones
Test audio output: speaker-test -t wav -c 2
Adjust volume: Set to comfortable level for conversation

Starting the Robot:
bash# Full system launch
roslaunch robot_controller emotional_robot.launch

# Debug mode (with visual feedback)
roslaunch robot_controller emotional_robot.launch debug:=true

# Quiet mode (minimal logging)
roslaunch robot_controller emotional_robot.launch quiet:=true
Stopping the Robot:
bash# Graceful shutdown
rosnode kill -a

# Or press Ctrl+C in the terminal
Checking Robot Status:
bash# See active nodes
rosnode list

# Check topics
rostopic list

# Monitor emotion detection
rostopic echo /emotion/current_emotion
