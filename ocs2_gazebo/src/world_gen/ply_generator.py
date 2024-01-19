from shapes import Shape

from typing import List

PLY_HEADER = """ply
format ascii 1.0
element vertex {}
property float x      
property float y           
property float z 
element face {}  
property list uchar int vertex_index
end_header
"""


class PlyGenerator:
    def generate_ply(self, filename: str, shapes: List[Shape]):
        header = PLY_HEADER.format(
            self._count_vertices(shapes), self._count_faces(shapes)
        )

        with open(filename, "w") as f:
            f.write(header)

            # Generate vertices
            for shape in shapes:
                for vertex in shape.get_vertices():
                    f.write(vertex.ply())

            # Generate faces
            offset = 0
            for shape in shapes:
                for face in shape.get_faces():
                    f.write(face.ply(offset))
                offset += shape.n_vertices

    def _count_vertices(self, shapes: List[Shape]):
        return sum([shape.n_vertices for shape in shapes])

    def _count_faces(self, shapes: List[Shape]):
        return sum([shape.n_faces for shape in shapes])