# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kinova_msgs/Arm_KinovaPoseGoal.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kinova_msgs.msg

class Arm_KinovaPoseGoal(genpy.Message):
  _md5sum = "a3d0acc7643d70196e513c76ce4fd6d9"
  _type = "kinova_msgs/Arm_KinovaPoseGoal"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Goal
KinovaPose kinova_pose

================================================================================
MSG: kinova_msgs/KinovaPose
float32 X
float32 Y
float32 Z
float32 ThetaX
float32 ThetaY
float32 ThetaZ"""
  __slots__ = ['kinova_pose']
  _slot_types = ['kinova_msgs/KinovaPose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       kinova_pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Arm_KinovaPoseGoal, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.kinova_pose is None:
        self.kinova_pose = kinova_msgs.msg.KinovaPose()
    else:
      self.kinova_pose = kinova_msgs.msg.KinovaPose()

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
      buff.write(_get_struct_6f().pack(_x.kinova_pose.X, _x.kinova_pose.Y, _x.kinova_pose.Z, _x.kinova_pose.ThetaX, _x.kinova_pose.ThetaY, _x.kinova_pose.ThetaZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.kinova_pose is None:
        self.kinova_pose = kinova_msgs.msg.KinovaPose()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.kinova_pose.X, _x.kinova_pose.Y, _x.kinova_pose.Z, _x.kinova_pose.ThetaX, _x.kinova_pose.ThetaY, _x.kinova_pose.ThetaZ,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.kinova_pose.X, _x.kinova_pose.Y, _x.kinova_pose.Z, _x.kinova_pose.ThetaX, _x.kinova_pose.ThetaY, _x.kinova_pose.ThetaZ))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.kinova_pose is None:
        self.kinova_pose = kinova_msgs.msg.KinovaPose()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.kinova_pose.X, _x.kinova_pose.Y, _x.kinova_pose.Z, _x.kinova_pose.ThetaX, _x.kinova_pose.ThetaY, _x.kinova_pose.ThetaZ,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
