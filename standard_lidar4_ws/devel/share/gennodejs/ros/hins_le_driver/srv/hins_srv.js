// Auto-generated. Do not edit!

// (in-package hins_le_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class hins_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.channel = null;
    }
    else {
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hins_srvRequest
    // Serialize message field [channel]
    bufferOffset = _serializer.int64(obj.channel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hins_srvRequest
    let len;
    let data = new hins_srvRequest(null);
    // Deserialize message field [channel]
    data.channel = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hins_le_driver/hins_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8850208bd0bd330676886afebd02f977';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 channel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hins_srvRequest(null);
    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = 0
    }

    return resolved;
    }
};

class hins_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.area1 = null;
      this.area2 = null;
      this.area3 = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('area1')) {
        this.area1 = initObj.area1
      }
      else {
        this.area1 = false;
      }
      if (initObj.hasOwnProperty('area2')) {
        this.area2 = initObj.area2
      }
      else {
        this.area2 = false;
      }
      if (initObj.hasOwnProperty('area3')) {
        this.area3 = initObj.area3
      }
      else {
        this.area3 = false;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hins_srvResponse
    // Serialize message field [area1]
    bufferOffset = _serializer.bool(obj.area1, buffer, bufferOffset);
    // Serialize message field [area2]
    bufferOffset = _serializer.bool(obj.area2, buffer, bufferOffset);
    // Serialize message field [area3]
    bufferOffset = _serializer.bool(obj.area3, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hins_srvResponse
    let len;
    let data = new hins_srvResponse(null);
    // Deserialize message field [area1]
    data.area1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [area2]
    data.area2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [area3]
    data.area3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hins_le_driver/hins_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5fd013987c49e2331580b63f581e6836';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool area1
    bool area2
    bool area3
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hins_srvResponse(null);
    if (msg.area1 !== undefined) {
      resolved.area1 = msg.area1;
    }
    else {
      resolved.area1 = false
    }

    if (msg.area2 !== undefined) {
      resolved.area2 = msg.area2;
    }
    else {
      resolved.area2 = false
    }

    if (msg.area3 !== undefined) {
      resolved.area3 = msg.area3;
    }
    else {
      resolved.area3 = false
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: hins_srvRequest,
  Response: hins_srvResponse,
  md5sum() { return '1b7a8b465a26f7d507ff8e5984967a01'; },
  datatype() { return 'hins_le_driver/hins_srv'; }
};
