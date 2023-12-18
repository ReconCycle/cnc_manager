ROS Action Server and Action Client to enable usage of the CNC machine within the ReconCycle workcell.

Server-side commands (on raspberry):
chmod +x build.sh
# Build Docker image
./build.sh
# Start stand-alone CNC action SERVER.
# Note that services for Pneumatics Pins must be running. (run compose file in /home/pi/compose-files, since it starts both CNC and pin manager containers)
docker-compose up
