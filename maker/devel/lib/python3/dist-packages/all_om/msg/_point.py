# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from all_om/point.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class point(genpy.Message):
  _md5sum = "0af1c687bcb24ea84d8c64d9f6633331"
  _type = "all_om/point"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 index
float32 x
float32 y
float32 z
float32 distance
float32 degree
int32 x_size
int32 y_size
string object
"""
  __slots__ = ['index','x','y','z','distance','degree','x_size','y_size','object']
  _slot_types = ['int32','float32','float32','float32','float32','float32','int32','int32','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       index,x,y,z,distance,degree,x_size,y_size,object

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(point, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.index is None:
        self.index = 0
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.distance is None:
        self.distance = 0.
      if self.degree is None:
        self.degree = 0.
      if self.x_size is None:
        self.x_size = 0
      if self.y_size is None:
        self.y_size = 0
      if self.object is None:
        self.object = ''
    else:
      self.index = 0
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.distance = 0.
      self.degree = 0.
      self.x_size = 0
      self.y_size = 0
      self.object = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i5f2i().pack(_x.index, _x.x, _x.y, _x.z, _x.distance, _x.degree, _x.x_size, _x.y_size))
      _x = self.object
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.index, _x.x, _x.y, _x.z, _x.distance, _x.degree, _x.x_size, _x.y_size,) = _get_struct_i5f2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.object = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i5f2i().pack(_x.index, _x.x, _x.y, _x.z, _x.distance, _x.degree, _x.x_size, _x.y_size))
      _x = self.object
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.index, _x.x, _x.y, _x.z, _x.distance, _x.degree, _x.x_size, _x.y_size,) = _get_struct_i5f2i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.object = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i5f2i = None
def _get_struct_i5f2i():
    global _struct_i5f2i
    if _struct_i5f2i is None:
        _struct_i5f2i = struct.Struct("<i5f2i")
    return _struct_i5f2i
