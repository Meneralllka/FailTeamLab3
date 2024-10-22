
"use strict";

let SetTorqueLimit = require('./SetTorqueLimit.js')
let StartController = require('./StartController.js')
let TorqueEnable = require('./TorqueEnable.js')
let RestartController = require('./RestartController.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let StopController = require('./StopController.js')
let SetSpeed = require('./SetSpeed.js')

module.exports = {
  SetTorqueLimit: SetTorqueLimit,
  StartController: StartController,
  TorqueEnable: TorqueEnable,
  RestartController: RestartController,
  SetCompliancePunch: SetCompliancePunch,
  SetComplianceMargin: SetComplianceMargin,
  SetComplianceSlope: SetComplianceSlope,
  StopController: StopController,
  SetSpeed: SetSpeed,
};
