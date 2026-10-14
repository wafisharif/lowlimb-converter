# Added by lowlimb-converter (2026-10-12): not in MyoConverter @ cadf380 (Station 6, change 6.2).
""" Contains the `PlanarJoint` parser: converted as the equivalent CustomJoint (see _as_custom.py). """

from myoconverter.xml.joints._as_custom import CustomJointEquivalent


class PlanarJoint(CustomJointEquivalent):
  """ OpenSim `PlanarJoint` -> the CustomJoint it is (lowlimb-converter, change 6.2). """
