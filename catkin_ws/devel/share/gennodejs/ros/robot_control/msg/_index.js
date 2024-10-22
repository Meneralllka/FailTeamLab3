
"use strict";

let contact = require('./contact.js');
let fsrInput = require('./fsrInput.js');
let newtactile = require('./newtactile.js');
let tactile = require('./tactile.js');
let accelerometr = require('./accelerometr.js');
let state = require('./state.js');
let coord = require('./coord.js');
let rigid = require('./rigid.js');

module.exports = {
  contact: contact,
  fsrInput: fsrInput,
  newtactile: newtactile,
  tactile: tactile,
  accelerometr: accelerometr,
  state: state,
  coord: coord,
  rigid: rigid,
};
