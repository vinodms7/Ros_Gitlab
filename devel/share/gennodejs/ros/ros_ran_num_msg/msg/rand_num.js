// Auto-generated. Do not edit!

// (in-package ros_ran_num_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class rand_num {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.number1 = null;
      this.number2 = null;
    }
    else {
      if (initObj.hasOwnProperty('number1')) {
        this.number1 = initObj.number1
      }
      else {
        this.number1 = 0;
      }
      if (initObj.hasOwnProperty('number2')) {
        this.number2 = initObj.number2
      }
      else {
        this.number2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rand_num
    // Serialize message field [number1]
    bufferOffset = _serializer.uint32(obj.number1, buffer, bufferOffset);
    // Serialize message field [number2]
    bufferOffset = _serializer.uint32(obj.number2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rand_num
    let len;
    let data = new rand_num(null);
    // Deserialize message field [number1]
    data.number1 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [number2]
    data.number2 = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_ran_num_msg/rand_num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20e27c3e6eee8dfccd76499464b39a05';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 number1
    uint32 number2
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rand_num(null);
    if (msg.number1 !== undefined) {
      resolved.number1 = msg.number1;
    }
    else {
      resolved.number1 = 0
    }

    if (msg.number2 !== undefined) {
      resolved.number2 = msg.number2;
    }
    else {
      resolved.number2 = 0
    }

    return resolved;
    }
};

module.exports = rand_num;
