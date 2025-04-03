// Auto-generated. Do not edit!

// (in-package all_om.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class point {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.distance = null;
      this.degree = null;
      this.x_size = null;
      this.y_size = null;
      this.object = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('degree')) {
        this.degree = initObj.degree
      }
      else {
        this.degree = 0.0;
      }
      if (initObj.hasOwnProperty('x_size')) {
        this.x_size = initObj.x_size
      }
      else {
        this.x_size = 0;
      }
      if (initObj.hasOwnProperty('y_size')) {
        this.y_size = initObj.y_size
      }
      else {
        this.y_size = 0;
      }
      if (initObj.hasOwnProperty('object')) {
        this.object = initObj.object
      }
      else {
        this.object = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type point
    // Serialize message field [index]
    bufferOffset = _serializer.int32(obj.index, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float32(obj.z, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [degree]
    bufferOffset = _serializer.float32(obj.degree, buffer, bufferOffset);
    // Serialize message field [x_size]
    bufferOffset = _serializer.int32(obj.x_size, buffer, bufferOffset);
    // Serialize message field [y_size]
    bufferOffset = _serializer.int32(obj.y_size, buffer, bufferOffset);
    // Serialize message field [object]
    bufferOffset = _serializer.string(obj.object, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type point
    let len;
    let data = new point(null);
    // Deserialize message field [index]
    data.index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [degree]
    data.degree = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_size]
    data.x_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_size]
    data.y_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [object]
    data.object = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.object);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'all_om/point';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0af1c687bcb24ea84d8c64d9f6633331';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 index
    float32 x
    float32 y
    float32 z
    float32 distance
    float32 degree
    int32 x_size
    int32 y_size
    string object
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new point(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.degree !== undefined) {
      resolved.degree = msg.degree;
    }
    else {
      resolved.degree = 0.0
    }

    if (msg.x_size !== undefined) {
      resolved.x_size = msg.x_size;
    }
    else {
      resolved.x_size = 0
    }

    if (msg.y_size !== undefined) {
      resolved.y_size = msg.y_size;
    }
    else {
      resolved.y_size = 0
    }

    if (msg.object !== undefined) {
      resolved.object = msg.object;
    }
    else {
      resolved.object = ''
    }

    return resolved;
    }
};

module.exports = point;
