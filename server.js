const express = require('express');
const app = require('./app');
const apiRouter = require('./routes/api');
const { spawn } = require('child_process');
const PORT = process.env.PORT || 5000;

// Middleware per gestire i dati JSON
app.use(express.json());

app.use('/api', apiRouter);

const startRos2Bridge = () => {
  const ros2BridgeProcess = spawn('bash', ['-c', 'ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./ros2/custom_bridge.yaml']);


  ros2BridgeProcess.stdout.on('data', (data) => {
    console.log(`ROS2 stdout: ${data}`);
  });

  ros2BridgeProcess.stderr.on('data', (data) => {
    console.error(`ROS2 stderr: ${data}`);
  });

  ros2BridgeProcess.on('close', (code) => {
    console.log(`ROS2 process exited with code ${code}`);
  });
};

const startCobottaToJointStates = () => {
  const cobottaToJointStates = spawn('python3', ['./ros2/nodes/cobotta_to_joint_states.py']);


  cobottaToJointStates.stdout.on('data', (data) => {
    console.log(`cobottaToJointStates stdout: ${data}`);
  });

  cobottaToJointStates.stderr.on('data', (data) => {
    console.error(`cobottaToJointStates stderr: ${data}`);
  });

  cobottaToJointStates.on('close', (code) => {
    console.log(`cobottaToJointStates process exited with code ${code}`);
  });
};

const startJointStatesToGazebo = () => {
  const jointsToGazebo = spawn('python3', ['./ros2/nodes/joint_states_to_gazebo.py']);


  jointsToGazebo.stdout.on('data', (data) => {
    console.log(`jointsToGazebo stdout: ${data}`);
  });

  jointsToGazebo.stderr.on('data', (data) => {
    console.error(`jointsToGazebo stderr: ${data}`);
  });

  jointsToGazebo.on('close', (code) => {
    console.log(`jointsToGazebo process exited with code ${code}`);
  });
};

const startJointStatesToCobotta = () => {
  const jointsToCobotta = spawn('python3', ['./ros2/nodes/joint_states_to_cobotta.py']);


  jointsToCobotta.stdout.on('data', (data) => {
    console.log(`jointsToCobotta stdout: ${data}`);
  });

  jointsToCobotta.stderr.on('data', (data) => {
    console.error(`jointsToCobotta stderr: ${data}`);
  });

  jointsToCobotta.on('close', (code) => {
    console.log(`jointsToCobotta process exited with code ${code}`);
  });
};

const startGazeboToJointStates = () => {
  const gazeboToJointStates = spawn('python3', ['./ros2/nodes/gazebo_to_joint_states.py']);


  gazeboToJointStates.stdout.on('data', (data) => {
    console.log(`gazeboToJointStates stdout: ${data}`);
  });

  gazeboToJointStates.stderr.on('data', (data) => {
    console.error(`gazeboToJointStates stderr: ${data}`);
  });

  gazeboToJointStates.on('close', (code) => {
    console.log(`gazeboToJointStates process exited with code ${code}`);
  });
};

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
  console.log("Launching bridge with custom name mapping...");

  startRos2Bridge();

  startCobottaToJointStates(); // read positions from Cobotta
  startJointStatesToGazebo(); // send the position to Gazebo

  startGazeboToJointStates(); // read position from Gazebo
  startJointStatesToCobotta(); // move Cobotta with positions received from Gazebo

});