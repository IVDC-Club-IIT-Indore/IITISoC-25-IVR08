
"use strict";

let LogEntry = require('./LogEntry.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let RadioStatus = require('./RadioStatus.js');
let Mavlink = require('./Mavlink.js');
let ManualControl = require('./ManualControl.js');
let HilSensor = require('./HilSensor.js');
let MountControl = require('./MountControl.js');
let CommandCode = require('./CommandCode.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let HilControls = require('./HilControls.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let Tunnel = require('./Tunnel.js');
let Altitude = require('./Altitude.js');
let RCIn = require('./RCIn.js');
let GPSRAW = require('./GPSRAW.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let LandingTarget = require('./LandingTarget.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let HomePosition = require('./HomePosition.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let HilGPS = require('./HilGPS.js');
let Waypoint = require('./Waypoint.js');
let StatusText = require('./StatusText.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let WaypointList = require('./WaypointList.js');
let BatteryStatus = require('./BatteryStatus.js');
let GPSINPUT = require('./GPSINPUT.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let LogData = require('./LogData.js');
let RTCM = require('./RTCM.js');
let ESCStatus = require('./ESCStatus.js');
let VFR_HUD = require('./VFR_HUD.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let State = require('./State.js');
let Thrust = require('./Thrust.js');
let ESCInfo = require('./ESCInfo.js');
let SysStatus = require('./SysStatus.js');
let VehicleInfo = require('./VehicleInfo.js');
let RTKBaseline = require('./RTKBaseline.js');
let Vibration = require('./Vibration.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let ActuatorControl = require('./ActuatorControl.js');
let FileEntry = require('./FileEntry.js');
let RCOut = require('./RCOut.js');
let Trajectory = require('./Trajectory.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let ExtendedState = require('./ExtendedState.js');
let TerrainReport = require('./TerrainReport.js');
let PositionTarget = require('./PositionTarget.js');
let ParamValue = require('./ParamValue.js');
let CellularStatus = require('./CellularStatus.js');
let Param = require('./Param.js');
let WaypointReached = require('./WaypointReached.js');
let GPSRTK = require('./GPSRTK.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let DebugValue = require('./DebugValue.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let AttitudeTarget = require('./AttitudeTarget.js');

module.exports = {
  LogEntry: LogEntry,
  ESCStatusItem: ESCStatusItem,
  RadioStatus: RadioStatus,
  Mavlink: Mavlink,
  ManualControl: ManualControl,
  HilSensor: HilSensor,
  MountControl: MountControl,
  CommandCode: CommandCode,
  PlayTuneV2: PlayTuneV2,
  HilControls: HilControls,
  MagnetometerReporter: MagnetometerReporter,
  Tunnel: Tunnel,
  Altitude: Altitude,
  RCIn: RCIn,
  GPSRAW: GPSRAW,
  HilStateQuaternion: HilStateQuaternion,
  LandingTarget: LandingTarget,
  CamIMUStamp: CamIMUStamp,
  EstimatorStatus: EstimatorStatus,
  ESCTelemetry: ESCTelemetry,
  HomePosition: HomePosition,
  OnboardComputerStatus: OnboardComputerStatus,
  HilGPS: HilGPS,
  Waypoint: Waypoint,
  StatusText: StatusText,
  CameraImageCaptured: CameraImageCaptured,
  WaypointList: WaypointList,
  BatteryStatus: BatteryStatus,
  GPSINPUT: GPSINPUT,
  OverrideRCIn: OverrideRCIn,
  ADSBVehicle: ADSBVehicle,
  ESCInfoItem: ESCInfoItem,
  LogData: LogData,
  RTCM: RTCM,
  ESCStatus: ESCStatus,
  VFR_HUD: VFR_HUD,
  WheelOdomStamped: WheelOdomStamped,
  State: State,
  Thrust: Thrust,
  ESCInfo: ESCInfo,
  SysStatus: SysStatus,
  VehicleInfo: VehicleInfo,
  RTKBaseline: RTKBaseline,
  Vibration: Vibration,
  TimesyncStatus: TimesyncStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  NavControllerOutput: NavControllerOutput,
  ActuatorControl: ActuatorControl,
  FileEntry: FileEntry,
  RCOut: RCOut,
  Trajectory: Trajectory,
  ESCTelemetryItem: ESCTelemetryItem,
  HilActuatorControls: HilActuatorControls,
  ExtendedState: ExtendedState,
  TerrainReport: TerrainReport,
  PositionTarget: PositionTarget,
  ParamValue: ParamValue,
  CellularStatus: CellularStatus,
  Param: Param,
  WaypointReached: WaypointReached,
  GPSRTK: GPSRTK,
  OpticalFlowRad: OpticalFlowRad,
  DebugValue: DebugValue,
  GlobalPositionTarget: GlobalPositionTarget,
  AttitudeTarget: AttitudeTarget,
};
