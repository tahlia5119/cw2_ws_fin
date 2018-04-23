
"use strict";

let AddSoundSource = require('./AddSoundSource.js')
let DeleteCO2Source = require('./DeleteCO2Source.js')
let AddCO2Source = require('./AddCO2Source.js')
let LoadMap = require('./LoadMap.js')
let DeleteThermalSource = require('./DeleteThermalSource.js')
let DeleteRfidTag = require('./DeleteRfidTag.js')
let AddRfidTag = require('./AddRfidTag.js')
let MoveRobot = require('./MoveRobot.js')
let AddThermalSource = require('./AddThermalSource.js')
let DeleteSoundSource = require('./DeleteSoundSource.js')
let LoadExternalMap = require('./LoadExternalMap.js')
let RegisterGui = require('./RegisterGui.js')

module.exports = {
  AddSoundSource: AddSoundSource,
  DeleteCO2Source: DeleteCO2Source,
  AddCO2Source: AddCO2Source,
  LoadMap: LoadMap,
  DeleteThermalSource: DeleteThermalSource,
  DeleteRfidTag: DeleteRfidTag,
  AddRfidTag: AddRfidTag,
  MoveRobot: MoveRobot,
  AddThermalSource: AddThermalSource,
  DeleteSoundSource: DeleteSoundSource,
  LoadExternalMap: LoadExternalMap,
  RegisterGui: RegisterGui,
};
