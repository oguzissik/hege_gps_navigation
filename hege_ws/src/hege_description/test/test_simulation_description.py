"""Static regression tests for the HEGE Gazebo model.

These tests intentionally avoid a running ROS / Gazebo instance. Runtime
acceptance is a separate step, but these checks prevent the two severe and
easy-to-miss configuration regressions that motivated them.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMETERS = PACKAGE_ROOT / "urdf" / "hege_parameters.xacro"
MODEL = PACKAGE_ROOT / "urdf" / "hege.urdf.xacro"


def _property_values(path: Path) -> dict[str, float]:
    root = ET.parse(path).getroot()
    namespace = "{http://www.ros.org/wiki/xacro}"
    return {
        element.attrib["name"]: float(element.attrib["value"])
        for element in root.iter(f"{namespace}property")
        if element.attrib.get("value", "").replace(".", "", 1).isdigit()
    }


def test_gps_horizontal_noise_is_converted_from_metres_to_degrees():
    values = _property_values(PARAMETERS)
    sigma_m = values["gps_horizontal_stddev_m"]
    metres_per_degree = values["metres_per_latitude_degree"]
    sigma_deg = sigma_m / metres_per_degree

    assert math.isclose(sigma_m, 2.0)
    assert 1.0e-5 < sigma_deg < 3.0e-5

    model_text = MODEL.read_text(encoding="utf-8")
    assert "gps_horizontal_stddev_m/metres_per_latitude_degree" in model_text
    assert "${gps_horizontal_stddev}</stddev>" not in model_text


def test_ground_truth_has_an_isolated_topic_name():
    root = ET.parse(MODEL).getroot()
    plugins = [element for element in root.iter("plugin")
               if element.attrib.get("name") == "gz::sim::systems::OdometryPublisher"]
    assert len(plugins) == 1

    plugin = plugins[0]
    assert plugin.findtext("odom_topic") == "/hege/ground_truth/odom"
    assert plugin.findtext("odom_frame") == "world"
    assert plugin.findtext("robot_base_frame") == "base_footprint"
