const express = require('express');
const bodyParser = require('body-parser');
const rclnodejs = require('rclnodejs');
const sensor_msgs = rclnodejs.require('sensor_msgs');
const std_msgs = rclnodejs.require('std_msgs');
const app = express();

app.use(bodyParser.json());


let node;
let publisherJointPosition;
let publisherCalibrateCommand;
let publisherStatusCobotta;

// Inizializza ROS2
async function initializeRclNodejs() {
    try {
        await rclnodejs.init();
        console.log('rclnodejs initialized');
    } catch (error) {
        console.error('Error initializing rclnodejs:', error);
        process.exit(1); // Esce dall'applicazione se non è possibile inizializzare rclnodejs
    }
}

initializeRclNodejs();

const { spawn } = require('child_process');

let physical_robot_controller_process;

exports.startRobotController = (req, res) => {
    if (physical_robot_controller_process && !physical_robot_controller_process.killed) {
        res.json({ message: 'Already connected to Cobotta.' });
        return;
    }

    physical_robot_controller_process = spawn('python3', ['./ros2/nodes/physical_robot_controller.py']);

    physical_robot_controller_process.stdout.on('data', (data) => {
        console.log(`stdout: ${data}`);
    });

    physical_robot_controller_process.stderr.on('data', (data) => {
        console.error(`stderr: ${data}`);
    });

    physical_robot_controller_process.on('close', (code) => {
        console.log(`child process exited with code ${code}`);
        physical_robot_controller_process = null; 
    });

    res.json({ message: 'Robot controller started' });
};

// receive joints position from front-end, then publish on topic "joint_statesWeb"
exports.moveJoints = async (req, res) => {
    const { joint1, joint2, joint3, joint4, joint5, joint6 } = req.body;

    console.log('Received joint values:', { joint1, joint2, joint3, joint4, joint5, joint6 });

    try {
        if (!node) {
            node = new rclnodejs.Node('backend_node');
        }

        if (!publisherJointPosition) {
            publisherJointPosition = node.createPublisher(sensor_msgs.msg.JointState, 'joint_statesWeb');
        }

        const jointStateMsg = new sensor_msgs.msg.JointState();
        jointStateMsg.header.stamp = new rclnodejs.Time();
        jointStateMsg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
        jointStateMsg.position = [joint1, joint2, joint3, joint4, joint5, joint6];
        jointStateMsg.velocity = [];
        jointStateMsg.effort = [];

        publisherJointPosition.publish(jointStateMsg);
        console.log('Published JointState message:', jointStateMsg);

        res.status(200).json({ message: 'Joint values received and published successfully.' });
    } catch (error) {
        console.error('Error publishing JointState message:', error);
        res.status(500).json({ error: 'Error publishing JointState message.' });
    }
};

exports.calibrateRobot = async (req, res) => {

    try {
        if (!physical_robot_controller_process) {
            res.json({ message: 'You need first to connect to Cobotta.' });
            return;
        }
        if (!node) {
            node = new rclnodejs.Node('backend_node');
        }

        if (!publisherCalibrateCommand) {
            publisherCalibrateCommand = node.createPublisher('std_msgs/msg/Bool', 'calibrate_command');
        }

        let boolMsg = new std_msgs.msg.Bool(); 
        boolMsg.data = true;
        publisherCalibrateCommand.publish(boolMsg);

        res.status(200).json({ message: 'Robot calibration initiated.' });
    } catch (error) {
        console.error('Error while trying to calibrate Cobotta', error);
        res.status(500).json({ error: 'Error while trying to calibrate Cobotta' });
    }
};

exports.changeMode = (req, res) => {
    const { mode } = req.body;
    
    try {
        if (!node) {
            node = new rclnodejs.Node('backend_node');
        }

        if (!publisherStatusCobotta) {
            publisherStatusCobotta = node.createPublisher('std_msgs/msg/Bool', 'activeStatusCobotta');
        }

        let boolMsg = new std_msgs.msg.Bool(); 
        if (mode === 'GazeboToCobotta') {
            // We dont want read positions from Cobotta and sending them back to Gazebo
            boolMsg.data = false;
        } else {
            // CobottaToGazebo
            boolMsg.data = true;
        }

        publisherStatusCobotta.publish(boolMsg);

        res.status(200).json({ message: 'Mode handled successfully' });
    } catch (error) {
        console.error('Error handling change of mode:', error);
        res.status(500).json({ error: 'Error handling change of mode.' });
    }

    console.log("Mode received= ", mode);
}

// exports.startRobotController = (req, res) => {
//     if (pythonProcess && !pythonProcess.killed) {
//         console.log('Robot controller is already running');
//         res.json({ message: 'Robot controller is already running' });
//         return;
//     }
//     const physical_robot_controller_process = spawn('python3', ['./ros2/physical_robot_controller.py']);

//     physical_robot_controller_process.stdout.on('data', (data) => {
//         console.log(`stdout: ${data}`);
//     });

//     physical_robot_controller_process.stderr.on('data', (data) => {
//         console.error(`stderr: ${data}`);
//     });

//     physical_robot_controller_process.on('close', (code) => {
//         console.log(`child process exited with code ${code}`);
//     });

//     res.json({ message: 'Robot controller started' });
// };