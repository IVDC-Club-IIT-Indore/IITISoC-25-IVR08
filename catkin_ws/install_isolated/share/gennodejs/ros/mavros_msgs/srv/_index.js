
"use strict";

let FileRemoveDir = require('./FileRemoveDir.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let CommandAck = require('./CommandAck.js')
let FileWrite = require('./FileWrite.js')
let FileMakeDir = require('./FileMakeDir.js')
let ParamSet = require('./ParamSet.js')
let FileClose = require('./FileClose.js')
let CommandTOL = require('./CommandTOL.js')
let FileRename = require('./FileRename.js')
let ParamPush = require('./ParamPush.js')
let LogRequestData = require('./LogRequestData.js')
let MountConfigure = require('./MountConfigure.js')
let CommandInt = require('./CommandInt.js')
let LogRequestList = require('./LogRequestList.js')
let FileTruncate = require('./FileTruncate.js')
let SetMode = require('./SetMode.js')
let FileRemove = require('./FileRemove.js')
let CommandLong = require('./CommandLong.js')
let ParamPull = require('./ParamPull.js')
let ParamGet = require('./ParamGet.js')
let WaypointPull = require('./WaypointPull.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointClear = require('./WaypointClear.js')
let FileList = require('./FileList.js')
let WaypointPush = require('./WaypointPush.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let SetMavFrame = require('./SetMavFrame.js')
let StreamRate = require('./StreamRate.js')
let CommandHome = require('./CommandHome.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let MessageInterval = require('./MessageInterval.js')
let CommandBool = require('./CommandBool.js')
let FileChecksum = require('./FileChecksum.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileOpen = require('./FileOpen.js')
let FileRead = require('./FileRead.js')

module.exports = {
  FileRemoveDir: FileRemoveDir,
  CommandTriggerInterval: CommandTriggerInterval,
  CommandAck: CommandAck,
  FileWrite: FileWrite,
  FileMakeDir: FileMakeDir,
  ParamSet: ParamSet,
  FileClose: FileClose,
  CommandTOL: CommandTOL,
  FileRename: FileRename,
  ParamPush: ParamPush,
  LogRequestData: LogRequestData,
  MountConfigure: MountConfigure,
  CommandInt: CommandInt,
  LogRequestList: LogRequestList,
  FileTruncate: FileTruncate,
  SetMode: SetMode,
  FileRemove: FileRemove,
  CommandLong: CommandLong,
  ParamPull: ParamPull,
  ParamGet: ParamGet,
  WaypointPull: WaypointPull,
  CommandVtolTransition: CommandVtolTransition,
  WaypointClear: WaypointClear,
  FileList: FileList,
  WaypointPush: WaypointPush,
  VehicleInfoGet: VehicleInfoGet,
  SetMavFrame: SetMavFrame,
  StreamRate: StreamRate,
  CommandHome: CommandHome,
  CommandTriggerControl: CommandTriggerControl,
  MessageInterval: MessageInterval,
  CommandBool: CommandBool,
  FileChecksum: FileChecksum,
  WaypointSetCurrent: WaypointSetCurrent,
  LogRequestEnd: LogRequestEnd,
  FileOpen: FileOpen,
  FileRead: FileRead,
};
