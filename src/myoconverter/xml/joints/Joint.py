# Modified by lowlimb-converter (2026-10-06), from MyoConverter @ cadf380: every MuJoCo joint now gets pos = the
# joint's child-frame offset, so hinges rotate about the joint centre as in OpenSim (Station 2, change 2.1).
""" Contains a higher level `Joint` parser.

@author: Aleksi Ikkala
"""

from abc import abstractmethod
from typing import final

from myoconverter.xml.parsers import IParser
from myoconverter.xml import config as cfg
from myoconverter.xml.joints.utils import lock_joint
from myoconverter.xml.utils import str2vec, vec2str


class Joint(IParser):
  """ A parent class to handle common parsing tasks for joints, like locking joints. """

  @abstractmethod
  def _parse(self, xml, socket_parent_frame, socket_child_frame, pointer):
    """ Joint-specific parsers must implement this method.

    Child classes must implement this method. This is called from the 'parse' method below. Add any calculations /
    conversions that weren't included in the default calculations / conversions below.

    :param xml: OpenSim joint actuator XML element
    :param socket_parent_frame: Parent frame socket
    :param socket_child_frame: Child frame socket
    :param pointer: A pointer to the MuJoCo XML file where this joint will be added
    :return: A list of MuJoCo XML joints, a list of joint parameters
    """

  @final
  def parse(self, xml, socket_parent_frame, socket_child_frame, pointer, root_body):
    """ This function handles general parsing and converting of joints.

    :param xml: OpenSim joint actuator XML element
    :param socket_parent_frame: Parent frame socket
    :param socket_child_frame: Child frame socket
    :param pointer: A pointer to the MuJoCo XML file where this joint will be added
    :param root_body: A list of MuJoCo XML joints, a list of joint parameters
    :return: None
    """

    # Do joint specific parsing
    m_joints, params = self._parse(xml, socket_parent_frame, socket_child_frame, pointer)

    # OpenSim rotates the child about the joint's child frame M, whose origin is p_BM in the body frame. MuJoCo
    # rotates about the joint's pos (body frame, default 0), so set it. Slide joints are unaffected by pos.
    child_position = str2vec(socket_child_frame.find("translation").text)
    if any(child_position != 0):
      for j in m_joints:
        j.attrib["pos"] = vec2str(child_position)

    # Create an equality constraint if the joint (or any of the subjoints) is locked
    for p in params:
      if "_locked" in p and p["_locked"]:
        lock_joint(p, cfg.M_EQUALITY)

    # Set stiffness and damping to zero for joint between ground and root body; not sure if armature should be set to
    # zero as well?
    if root_body:
      for j in m_joints:
          j.attrib.update({"damping": "0", "stiffness": "0"})
