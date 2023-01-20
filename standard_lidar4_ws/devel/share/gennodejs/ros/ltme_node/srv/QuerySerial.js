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

class QuerySerialRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuerySerialRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuerySerialRequest
    let len;
    let data = new QuerySerialRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltme_node/QuerySerialRequest';
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
    const resolved = new QuerySerialRequest(null);
    return resolved;
    }
};

class QuerySerialResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.serial = null;
    }
    else {
      if (initObj.hasOwnProperty('serial')) {
        this.serial = initObj.serial
      }
      else {
        this.serial = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QuerySerialResponse
    // Serialize message field [serial]
    bufferOffset = _serializer.string(obj.serial, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QuerySerialResponse
    let len;
    let data = new QuerySerialResponse(null);
    // Deserialize message field [serial]
    data.serial = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.serial);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ltme_node/QuerySerialResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fca40cf463282a80db4e2037c8a61741';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string serial
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QuerySerialResponse(null);
    if (msg.serial !== undefined) {
      resolved.serial = msg.serial;
    }
    else {
      resolved.serial = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: QuerySerialRequest,
  Response: QuerySerialResponse,
  md5sum() { return 'fca40cf463282a80db4e2037c8a61741'; },
  datatype() { return 'ltme_node/QuerySerial'; }
};
