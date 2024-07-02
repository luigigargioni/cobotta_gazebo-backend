const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robotController');

router.post('/move-joints', robotController.moveJoints);
router.post('/start-robot-controller', robotController.startRobotController);
router.post('/calibrate', robotController.calibrateRobot);
router.post('/mode', robotController.changeMode);
module.exports = router;