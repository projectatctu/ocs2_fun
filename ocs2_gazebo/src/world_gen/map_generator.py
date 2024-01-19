import lxml.etree as ET
from shapes import Shape, Box, ShapeTypes

import rospy
from visualization_msgs.msg import Marker, MarkerArray

from utils import xml_set_value

from typing import List


class MapGenerator:
    def __init__(self) -> None:
        """Initialize map generator with an empty list of shapes"""
        self.shapes: List[Shape] = list()

    def add_shape(self, shape: Shape) -> None:
        """Add a shape to the map

        Args:
            shape (Shape): Shape to be added
        """
        self.shapes.append(shape)

    def generate_map(self, filename: str) -> None:
        """Generate a gazebo world file with the given filename and previously added shapes

        Args:
            filename (str): Name of the world file
        """
        # Create root element
        root = ET.Element("sdf", version="1.6")
        world = ET.SubElement(root, "world", name="generated_world")

        # Add ground plane
        self._add_ground_plane(world)

        # Add the rest of shapes
        for shape in self.shapes:
            self._add_shape(world, shape)

        # Generate xml
        with open(filename, "wb") as f:
            f.write(ET.tostring(root, pretty_print=True))

    def _add_ground_plane(self, world: ET.SubElement) -> None:
        # xml_set_value(world, "include/uri", "model://ground_plane")
        pass

    def _add_shape(self, world: ET.SubElement, shape: Shape) -> None:
        xml_elem = shape.generate_xml()
        world.append(xml_elem)


class MapReader:
    def __init__(self, filename: str) -> None:
        """Read a world file

        Args:
            filename (str): world file path
        """
        self.filename = filename
        root = self._read()
        self._shapes = self._extract_shapes(root)

    def get_shapes(self) -> List[Shape]:
        """Return a list of extracted shapes

        Returns:
            List[Shape]: Shapes found in the world file
        """
        return self._shapes

    def _read(self):
        tree = ET.parse(self.filename)
        root = tree.getroot()
        return root

    def _extract_shapes(self, root: ET.Element) -> List[Shape]:
        shapes = list()
        shapes.extend(self._extract_boxes(root))
        return shapes

    def _extract_boxes(self, root: ET.Element) -> List[Box]:
        boxes = list()
        for model in root.iter("model"):
            shape_type = self._get_shape_type(model)
            if shape_type != ShapeTypes.BOX:
                continue

            name = model.attrib["name"]
            visualize = not name.endswith(Shape.DONT_VISUALIZE)
            if not visualize:
                name = name[: -len(Shape.DONT_VISUALIZE)]
            position = list(map(float, model.find("pose").text.split()[:3]))
            size = list(
                map(float, model.find("link/collision/geometry/box/size").text.split())
            )
            box = Box(name, *position, *size, visualize=visualize)
            boxes.append(box)
        return boxes

    def _get_shape_type(self, model: ET.Element) -> ShapeTypes:
        if model.findall("box") is not None:
            return ShapeTypes.BOX
