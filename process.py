import subprocess

# Run the first script, collecting the images
subprocess.run(['python', 'new.py'])

# Run the second script
subprocess.run(['python', 'c.py'])

for i in range(3):

    subprocess.run(['python', '/home/jetson/AutonomousPhotogrammetry/pvescOG/edit.py'])

    subprocess.run(['python', 'new.py'])

    subprocess.run(['python', 'c.py'])

subprocess.run(['python', 'victory.py'])

print("program ended")
