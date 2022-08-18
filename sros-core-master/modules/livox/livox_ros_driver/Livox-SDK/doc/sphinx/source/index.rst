.. Didar documentation master file, created by
   sphinx-quickstart on Mon Dec 10 15:38:07 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to the Livox LiDAR SDK API Reference
#############################################

.. toctree::
   :maxdepth: 2
   :numbered:
   :caption: Contents:

Basic Types and Functions
#############################

.. doxygenenum:: DeviceType
   :project: didar

.. doxygenenum:: LidarState
   :project: didar

.. doxygenenum:: LidarFeature
   :project: didar

.. doxygenenum:: LidarIpMode
   :project: didar

.. doxygenenum:: LivoxStatus
   :project: didar

.. doxygentypedef:: livox_status
   :project: didar

.. doxygenenum:: KeyErrorCode
   :project: didar

.. doxygenenum:: DeviceParamKeyName
   :project: didar

.. doxygenstruct:: KeyValueParam
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: DeviceParameterResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: GetDeviceParameterRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: GetDeviceParameterResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygenenum:: DeviceEvent
   :project: didar

.. doxygenenum:: TimestampType
   :project: didar

.. doxygenenum:: PointDataType
   :project: didar

.. doxygenenum:: PointCloudReturnMode
   :project: didar

.. doxygenenum:: ImuFreq
   :project: didar

.. doxygenstruct:: LivoxRawPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxSpherPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxExtendRawPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxExtendSpherPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxDualExtendRawPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxDualExtendSpherPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxImuPoint
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: DeviceInfo
   :project: didar
   :members:
   :undoc-members:

.. doxygenunion:: StatusUnion
  :project: didar

.. doxygenstruct:: ReturnCode
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LivoxSdkVersion
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: GetLivoxSdkVersion
   :project: didar

.. doxygenfunction:: Init
   :project: didar

.. doxygenfunction:: Start
   :project: didar

.. doxygenfunction:: Uninit
   :project: didar

.. doxygenstruct:: BroadcastDeviceInfo
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: DeviceBroadcastCallback
  :project: didar

.. doxygenfunction:: SetBroadcastCallback
   :project: didar

.. doxygentypedef:: DeviceStateUpdateCallback
   :project: didar

.. doxygenfunction:: SetDeviceStateUpdateCallback
   :project: didar

.. doxygenfunction:: AddHubToConnect
   :project: didar

.. doxygenfunction:: AddLidarToConnect
   :project: didar

.. doxygenfunction:: GetConnectedDevices
   :project: didar

General Functions
######################

Query Device Information
==========================

.. doxygenstruct:: DeviceInformationResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: DeviceInformationCallback
   :project: didar

.. doxygenfunction:: QueryDeviceInformation
   :project: didar

Receive Point Cloud Data
==========================

.. doxygenstruct:: LivoxEthPacket
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: DataCallback
   :project: didar

.. doxygenfunction:: SetDataCallback
   :project: didar

.. doxygenfunction:: HubGetLidarHandle
   :project: didar

Set Coordinate System
============================

.. doxygenfunction:: SetCartesianCoordinate
   :project: didar

.. doxygenfunction:: SetSphericalCoordinate
   :project: didar

Error Message From Device
=======================================

.. doxygenunion:: ErrorMessage
   :project: didar

.. doxygenstruct:: LidarErrorCode
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubErrorCode
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: ErrorMessageCallback
   :project: didar

.. doxygenfunction:: SetErrorMessageCallback
   :project: didar

Configure Static/Dynamic IP
=======================================

.. doxygenstruct:: SetDeviceIPModeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: SetStaticDynamicIP
   :project: didar

.. doxygenstruct:: SetStaticDeviceIpModeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: SetStaticIp
   :project: didar

.. doxygenfunction:: SetDynamicIp
   :project: didar

.. doxygenstruct:: GetDeviceIpModeResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: GetDeviceIpInformationCallback
   :project: didar

.. doxygenfunction:: GetDeviceIpInformation
   :project: didar

Disconnect Device
=======================================

.. doxygenfunction:: DisconnectDevice
   :project: didar

Reboot Device
=======================================

.. doxygenfunction:: RebootDevice
   :project: didar

Reset LiDAR/Hub's All Parameters
=======================================

.. doxygentypedef:: DeviceResetParametersCallback
   :project: didar

.. doxygenfunction:: DeviceResetAllParameters
   :project: didar

.. doxygenfunction:: DeviceResetParameters
   :project: didar

Livox Hub Functions
######################

Query Connected LiDAR Unit Information
=======================================

.. doxygenstruct:: ConnectedLidarInfo
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubQueryLidarInformationResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubQueryLidarInformationCallback
   :project: didar

Configure LiDAR Unit Mode
==========================================

.. doxygenstruct:: HubSetModeResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubSetModeCallback
   :project: didar

.. doxygenstruct:: HubSetModeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: LidarModeRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: HubSetMode
   :project: didar

Query LiDAR Unit Status
==========================================

.. doxygenstruct:: LidarStateItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubQueryLidarStatusResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubQueryLidarStatusCallback
   :project: didar

.. doxygenfunction:: HubQueryLidarStatus
   :project: didar

Sampling Control
=================

.. doxygentypedef:: CommonCommandCallback
   :project: didar

.. doxygenfunction:: HubStartSampling
   :project: didar

.. doxygenfunction:: HubStopSampling
   :project: didar

Slot Power Control
========================

.. doxygenstruct:: HubControlSlotPowerRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: HubControlSlotPower
   :project: didar

.. doxygenstruct:: HubQuerySlotPowerStatusResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubQuerySlotPowerStatusCallback
   :project: didar

.. doxygenfunction:: HubQuerySlotPowerStatus
   :project: didar

Configure Livox Hub Extrinsic Parameters
========================================

.. doxygenstruct:: HubSetExtrinsicParameterResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubSetExtrinsicParameterCallback
   :project: didar

.. doxygenstruct:: HubSetExtrinsicParameterRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: ExtrinsicParameterRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: HubSetExtrinsicParameter
   :project: didar

.. doxygenstruct:: HubGetExtrinsicParameterRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: DeviceBroadcastCode
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetExtrinsicParameterResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: ExtrinsicParameterResponseItem
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubGetExtrinsicParameterCallback
   :project: didar

.. doxygenfunction:: HubGetExtrinsicParameter
   :project: didar

Enable Hub Calculating Extrinsic Parameters
========================================================

.. doxygenfunction:: HubExtrinsicParameterCalculation
   :project: didar

Enable or Disable The Rain/Fog Suppression
========================================================

.. doxygenstruct:: RainFogSuppressRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubRainFogSuppressRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubRainFogSuppressResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubRainFogSuppressCallback
   :project: didar

.. doxygenfunction:: HubRainFogSuppress
   :project: didar

Turn On or Off Fan of LiDAR Unit 
========================================================

.. doxygenstruct:: FanControlRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubFanControlRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubFanControlResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubFanControlCallback
   :project: didar

.. doxygenfunction:: HubFanControl
   :project: didar

.. doxygenstruct:: GetFanStateRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetFanStateRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: GetFanStateResponseItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetFanStateResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubGetFanStateCallback
   :project: didar

.. doxygenfunction:: HubGetFanState
   :project: didar

Config Point Cloud Return Mode of LiDAR Unit
========================================================

.. doxygenstruct:: SetPointCloudReturnModeRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubSetPointCloudReturnModeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubSetPointCloudReturnModeResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubSetPointCloudReturnModeCallback
   :project: didar

.. doxygenfunction:: HubSetPointCloudReturnMode
   :project: didar

.. doxygenstruct:: GetPointCloudReturnModeRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetPointCloudReturnModeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: GetPointCloudReturnModeResponseItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetPointCloudReturnModeResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubGetPointCloudReturnModeCallback
   :project: didar

.. doxygenfunction:: HubGetPointCloudReturnMode
   :project: didar

Config IMU Push Frequency of LiDAR Unit
========================================================

.. doxygenstruct:: SetImuPushFrequencyRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubSetImuPushFrequencyRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubSetImuPushFrequencyResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubSetImuPushFrequencyCallback
   :project: didar

.. doxygenfunction:: HubSetImuPushFrequency
   :project: didar

.. doxygenstruct:: GetImuPushFrequencyRequestItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetImuPushFrequencyRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: GetImuPushFrequencyResponseItem
   :project: didar
   :members:
   :undoc-members:

.. doxygenstruct:: HubGetImuPushFrequencyResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: HubGetImuPushFrequencyCallback
   :project: didar

.. doxygenfunction:: HubGetImuPushFrequency
   :project: didar

LiDAR Functions
#####################

Configure LiDAR Mode
==============================

.. doxygenenum:: LidarMode
   :project: didar

.. doxygenfunction:: LidarSetMode
   :project: didar

Sample Control
=================

.. doxygenfunction:: LidarStartSampling
   :project: didar

.. doxygenfunction:: LidarStopSampling
   :project: didar

Configure LiDAR Extrinsic Parameters
====================================

.. doxygenstruct:: LidarSetExtrinsicParameterRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: LidarSetExtrinsicParameter
   :project: didar

.. doxygenstruct:: LidarGetExtrinsicParameterResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: LidarGetExtrinsicParameterCallback
   :project: didar

.. doxygenfunction:: LidarGetExtrinsicParameter
   :project: didar

Enable and Disable the Rain/Fog Suppression
============================================

.. doxygenfunction:: LidarRainFogSuppress
   :project: didar

Turn On or Off LiDAR's Fan
============================================

.. doxygenfunction:: LidarTurnOnFan
   :project: didar

.. doxygenfunction:: LidarTurnOffFan
   :project: didar

.. doxygenstruct:: LidarGetFanStateResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: LidarGetFanStateCallback
   :project: didar

.. doxygenfunction:: LidarGetFanState
   :project: didar

Config LiDAR's Point Cloud Return Mode
============================================

.. doxygenfunction:: LidarSetPointCloudReturnMode
   :project: didar

.. doxygenstruct:: LidarGetPointCloudReturnModeResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: LidarGetPointCloudReturnModeCallback
   :project: didar

.. doxygenfunction:: LidarGetPointCloudReturnMode
   :project: didar

Config LiDAR's IMU Push Frequency
============================================

.. doxygenfunction:: LidarSetImuPushFrequency
   :project: didar

.. doxygenstruct:: LidarGetImuPushFrequencyResponse
   :project: didar
   :members:
   :undoc-members:

.. doxygentypedef:: LidarGetImuPushFrequencyCallback
   :project: didar

.. doxygenfunction:: LidarGetImuPushFrequency
   :project: didar

Config LiDAR's UTC Sychronization
============================================

.. doxygenfunction:: LidarSetRmcSyncTime
   :project: didar

.. doxygenstruct:: LidarSetUtcSyncTimeRequest
   :project: didar
   :members:
   :undoc-members:

.. doxygenfunction:: LidarSetUtcSyncTime
   :project: didar

Enable or Disable LiDAR HighSensetivity Mode
=======================================

.. doxygenfunction:: LidarEnableHighSensitivity
   :project: didar

.. doxygenfunction:: LidarDisableHighSensitivity
   :project: didar

.. doxygenfunction:: LidarGetHighSensitivityState
   :project: didar

Config LiDAR's Scan Pattern
============================================

.. doxygenenum:: LidarScanPattern
   :project: didar

.. doxygenfunction:: LidarSetScanPattern
   :project: didar

.. doxygenfunction:: LidarGetScanPattern
   :project: didar

Config LiDAR's Slot Number
============================================

.. doxygenfunction:: LidarSetSlotNum
   :project: didar

.. doxygenfunction:: LidarGetSlotNum
   :project: didar
