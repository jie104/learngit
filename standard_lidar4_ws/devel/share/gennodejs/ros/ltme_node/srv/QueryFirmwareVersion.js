// Auto-generated. Do not edit!

// (in-package ltme_node.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class QueryFirmwareVersionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QueryFirmwareVersionRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QueryFirmwareVersionRequest
    let len;
    let data = new QueryFirmwareVersionRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltme_node/QueryFirmwareVersionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QueryFirmwareVersionRequest(null);
    return resolved;
    }
};

class QueryFirmwareVersionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.firmware_version = null;
    }
    else {
      if (initObj.hasOwnProperty('firmware_version')) {
        this.firmware_version = initObj.firmware_version
      }
      else {
        this.firmware_version = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QueryFirmwareVersionResponse
    // Serialize message field [firmware_version]
    bufferOffset = _serializer.string(obj.firmware_version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QueryFirmwareVersionResponse
    let len;
    let data = new QueryFirmwareVersionResponse(null);
    // Deserialize message field [firmware_version]
    data.firmware_version = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.firmware_version);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltme_node/QueryFirmwareVersionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '968367e081bb6dba33b3daf3e01dab62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string firmware_version
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QueryFirmwareVersionResponse(null);
    if (msg.firmware_version !== undefined) {
      resolved.firmware_version = msg.firmware_version;
    }
    else {
      resolved.firmware_version = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: QueryFirmwareVersionRequest,
  Response: QueryFirmwareVersionResponse,
  md5sum() { return '968367e081bb6dba33b3daf3e01dab62'; },
  datatype() { return 'ltme_node/QueryFirmwareVersion'; }
};
