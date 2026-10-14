# Modified by lowlimb-converter (2026-10-10), from MyoConverter @ cadf380: keeps the OpenSim file path (OSIM_FILE)
# so path points can be evaluated with OpenSim kinematics (Station 3, change 3.1).
# Modified by lowlimb-converter (2026-10-12): the model's <defaults> objects are applied to every component that omits
# a property, as OpenSim itself does when it reads the file (MyoConverter limitation 3; change 1c.3).
# Modified by lowlimb-converter (2026-10-12): the model is read with read_osim(), so files with "::" element names load
# (change 6.1). A model with its own contact forces gets no extra ground plane (change 6.6).
""" Contains definitions for global variables that are used in the conversion process.

@author: Aleksi Ikkala
"""
import os
from lxml import etree

from loguru import logger

from myoconverter.xml.utils import read_osim
from myoconverter.xml.parsers import BodyParser, ConstraintParser, ForceParser, JointParser, PathPointParser, \
  PathWrapParser, PathWrapSetParser, WrapObjectParser, MarkerParser


# Variables for paths, model names
OUTPUT_FOLDER = None
GEOMETRY_FOLDER = None
OUTPUT_GEOMETRY_FOLDER = None
MODEL_NAME = None
OUTPUT_XML_FILE = None
OUTPUT_LOG_FILE = None
OUTPUT_PLOT_FOLDER = None

# Parsing parameters
ADD_GROUND_GEOM = None
CONTACT_GROUPS = None  # lowlimb-converter (change 6.6): contact pairs grouped by OpenSim material, for the calibration
TREAT_AS_NORMAL_PATH_POINT = None

# Variables for parsed OpenSim XML
OSIM_FILE = None  # lowlimb-converter: path of the OpenSim model file being converted
OPENSIM = None
O_GROUND = None
O_BODYSET = None
O_JOINTSET = None
O_FORCESET = None
O_CONSTRAINTSET = None

# Variables for parsed MuJoCo XML
MUJOCO = None
M_WORLDBODY = None
M_GROUND = None
M_ASSET = None
M_TENDON = None
M_ACTUATOR = None
M_EQUALITY = None
M_CONTACT = None

# Variables for parsers
BODY_PARSER = None
CONSTRAINT_PARSER = None
FORCE_PARSER = None
JOINT_PARSER = None
PATH_POINT_PARSER = None
PATH_WRAP_PARSER = None
PATH_WRAP_SET_PARSER = None
WRAP_OBJECT_PARSER = None
MARKER_PARSER = None

# Some URLs
URL_API = None
URL_API_XML = None
URL_HOME = None


def initialise(xml_file, output_folder, **kwargs):
  """ Initialise the configs / global variables.

  :param xml_file: The OpenSim XML file
  :param output_folder: Output folder where the MuJoCo model will be saved into
  :param kwargs: Optional keyword arguments
  :return: None
  """

  # Set folder paths
  global OUTPUT_FOLDER, GEOMETRY_FOLDER, OUTPUT_GEOMETRY_FOLDER, MODEL_NAME, OUTPUT_XML_FILE, OUTPUT_LOG_FILE, \
    OUTPUT_PLOT_FOLDER
  OUTPUT_FOLDER = output_folder
  OUTPUT_GEOMETRY_FOLDER = os.path.join(OUTPUT_FOLDER, "Geometry")
  MODEL_NAME = os.path.split(xml_file)[1][:-5]
  OUTPUT_XML_FILE = os.path.join(OUTPUT_FOLDER, MODEL_NAME + "_cvt1.xml")
  # OUTPUT_LOG_FILE = os.path.join(OUTPUT_FOLDER, f"{MODEL_NAME}-conversion.log")
  OUTPUT_PLOT_FOLDER = os.path.join(OUTPUT_FOLDER, "Step1_xmlConvert")

  # Set URLs
  global URL_API, URL_API_XML, URL_HOME
  URL_HOME = "https://myoconverter.readthedocs.io/en/latest/"
  URL_API = "https://myoconverter.readthedocs.io/en/latest/autoapi/index.html"
  URL_API_XML = "https://myoconverter.readthedocs.io/en/latest/autoapi/myoconverter/xml/index.html"

  if "geometry_folder" not in kwargs or kwargs["geometry_folder"] is None:
    GEOMETRY_FOLDER = os.path.join(os.path.dirname(xml_file), "Geometry")
  else:
    GEOMETRY_FOLDER = kwargs["geometry_folder"]

  # Create the output and geometry folders
  os.makedirs(OUTPUT_FOLDER, exist_ok=True)
  os.makedirs(OUTPUT_GEOMETRY_FOLDER, exist_ok=True)

  # Create an XML parser
  parser = etree.XMLParser(remove_blank_text=True)

  # Set parsing parameters
  global ADD_GROUND_GEOM, CONTACT_GROUPS
  ADD_GROUND_GEOM = kwargs.get("add_ground_geom", False)
  CONTACT_GROUPS = {}

  # Optional parameters for users
  global TREAT_AS_NORMAL_PATH_POINT
  TREAT_AS_NORMAL_PATH_POINT = kwargs.get("treat_as_normal_path_point", False)

  # Read and parse OpenSim model
  global OPENSIM, O_GROUND, O_BODYSET, O_JOINTSET, O_FORCESET, O_CONSTRAINTSET, O_MARKERSET, OSIM_FILE
  OSIM_FILE = os.path.abspath(xml_file)
  OPENSIM = read_osim(xml_file, parser).getroot().find("Model")   # lowlimb-converter (change 6.1)
  apply_document_defaults(OPENSIM)   # lowlimb-converter (change 1c.3)
  O_GROUND = OPENSIM.find("Ground")
  O_BODYSET = OPENSIM.find("BodySet/objects")
  O_BODYSET = [] if O_BODYSET is None else O_BODYSET
  O_JOINTSET = OPENSIM.find("JointSet/objects")
  O_JOINTSET = [] if O_JOINTSET is None else O_JOINTSET
  O_FORCESET = OPENSIM.find("ForceSet/objects")
  O_FORCESET = [] if O_FORCESET is None else O_FORCESET
  O_CONSTRAINTSET = OPENSIM.find("ConstraintSet/objects")
  O_CONSTRAINTSET = [] if O_CONSTRAINTSET is None else O_CONSTRAINTSET
  O_MARKERSET = OPENSIM.find("MarkerSet/objects")
  O_MARKERSET = [] if O_MARKERSET is None else O_MARKERSET

  # lowlimb-converter (change 6.6): a model with its own contact forces gets those contacts, not MyoConverter's ground
  # plane in contact with every mesh (which would add contacts OpenSim doesn't have)
  contact = [f for f in OPENSIM.findall("ForceSet/objects/HuntCrossleyForce")
             if f.find("appliesForce") is None or (f.find("appliesForce").text or "").strip() not in ("false", "0")]
  if ADD_GROUND_GEOM and contact:
    logger.warning("The model defines its own contact (HuntCrossleyForce): no ground plane is added")
    ADD_GROUND_GEOM = False

  # Read and parse MuJoCo template
  global MUJOCO, M_GROUND, M_WORLDBODY, M_ASSET, M_TENDON, M_ACTUATOR, M_EQUALITY, M_CONTACT
  MUJOCO = etree.parse(os.path.join(os.path.dirname(os.path.realpath(__file__)), "template.xml"), parser).getroot()
  M_WORLDBODY = MUJOCO.find("worldbody")
  M_GROUND = M_WORLDBODY.find("body[@name='ground']")
  M_ASSET = MUJOCO.find("asset")
  M_TENDON = MUJOCO.find("tendon")
  M_ACTUATOR = MUJOCO.find("actuator")
  M_EQUALITY = MUJOCO.find("equality")
  M_CONTACT = MUJOCO.find("contact")

  # Create all parsers
  # NOTE: Initialisation order matters here, because e.g BodyParser imports WRAP_OBJECT_PARSER, so it must be defined
  # first, and PathWrapSetParser imports PATH_WRAP_PARSER, so it must be defined first
  global BODY_PARSER, CONSTRAINT_PARSER, FORCE_PARSER, JOINT_PARSER, PATH_POINT_PARSER, PATH_WRAP_PARSER, \
    PATH_WRAP_SET_PARSER, WRAP_OBJECT_PARSER, MARKER_PARSER
  WRAP_OBJECT_PARSER = WrapObjectParser()
  PATH_WRAP_PARSER = PathWrapParser()
  PATH_WRAP_SET_PARSER = PathWrapSetParser()
  PATH_POINT_PARSER = PathPointParser()
  BODY_PARSER = BodyParser()
  FORCE_PARSER = ForceParser()
  JOINT_PARSER = JointParser()
  CONSTRAINT_PARSER = ConstraintParser()
  MARKER_PARSER = MarkerParser()


def apply_document_defaults(model):
  """ lowlimb-converter (change 1c.3): fill in properties a component omits from the file's <defaults> objects.

  When OpenSim reads a model file, an object whose XML leaves out a property takes that property from the
  document's default object of the same class (the <defaults> element), not from the class's built-in default:
  e.g. a CoordinateActuator without <optimal_force> in Rajagopal 2016 gets 300 from <defaults>, not 1. MyoConverter
  reads the XML directly and never looked at <defaults> (its documented limitation 3), so such a property took the
  converter's own fallback. This copies every omitted property of a component from the default object of its class,
  at every level (a nested object is filled from its own class's default object), before anything is parsed. Models
  without <defaults> are unchanged.

  :param model: the OpenSim `Model` XML element (modified in place)
  :return: number of properties filled in
  """
  from copy import deepcopy
  defaults_element = model.find("defaults")
  if defaults_element is None:
    return 0
  templates = {d.tag: d for d in defaults_element if isinstance(d.tag, str)}
  targets = [el for el in model.iter() if isinstance(el.tag, str) and el.tag in templates
             and el is not defaults_element and defaults_element not in el.iterancestors()]
  filled = 0
  for el in targets:
    present = {c.tag for c in el if isinstance(c.tag, str)}
    for prop in templates[el.tag]:
      if isinstance(prop.tag, str) and prop.tag not in present:
        el.append(deepcopy(prop))
        filled += 1
  if filled:
    logger.info(f"Filled in {filled} omitted properties from the model's <defaults> element, as OpenSim does")
  return filled
