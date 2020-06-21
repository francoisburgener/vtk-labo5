"""
VTK - HEIG - Lab05
authors: François Burgener, Tiago Povoa Quinteiro
"""
import math
import pyproj
import vtk
import numpy as np

# Constante file
VTK_PLANE_GPS = "vtkgps.txt"
TEXTURE_IMG = "glider_map.jpg"
VTK_MAP = "EarthEnv-DEM90_N60E010.bil"

# Constante MAP
DEGREE = 5
MIN_LAT = 60
MIN_LONG = 10
MAX_LAT = MIN_LAT + DEGREE
MAX_LONG = MIN_LONG + DEGREE
EARTH_RADIUS = 6371009
MAP_WIDTH = 6000

# Window parameters (with/height)
WINDOW_WIDTH_SIZE = 1000
WINDOW_HEIGTH_SIZE = 1000


def coordinate_earth(lat, lng, alt):
    """
    Shifts the altitude to match the earth's curvature
    (approximated as a sphere)
    :param lat: latitude
    :param lng: longitude
    :param alt: altitude
    :return: an x,y,z point transformed
    """

    transform = vtk.vtkTransform()
    transform.RotateY(lng)
    transform.RotateX(-lat)
    transform.Translate(0, 0, EARTH_RADIUS + alt)

    return transform.TransformPoint(0, 0, 0)


# https://pyproj4.github.io/pyproj/stable/gotchas.html#upgrading-to-pyproj-2-from-pyproj-1
def convert_rt90_to_gps_coordinate(x, y):
    transformer = pyproj.Transformer.from_crs("epsg:3021", "epsg:4326")
    return transformer.transform(y, x)


TOP_LEFT = convert_rt90_to_gps_coordinate(1349340, 7022573)
TOP_RIGHT = convert_rt90_to_gps_coordinate(1371573, 7022967)
BOTTOM_LEFT = convert_rt90_to_gps_coordinate(1349602, 7005969)
BOTTOM_RIGHT = convert_rt90_to_gps_coordinate(1371835, 7006362)


def read_txt(filename):
    """
    Reads the raw output data from a file.
    :param filename: the file name
    :return: TODO
    """
    with open(filename, 'r') as fd:
        size = fd.readline()
        lines = fd.readlines()

        coordinate = []

        for line in lines:
            tmp = line.split()
            x = int(tmp[1])
            y = int(tmp[2])
            z = float(tmp[3])

            coordinate.append((x, y, z))

        return size, coordinate


px = [BOTTOM_LEFT[1], BOTTOM_RIGHT[1], TOP_RIGHT[1], TOP_LEFT[1]]
py = [BOTTOM_LEFT[0], BOTTOM_RIGHT[0], TOP_RIGHT[0], TOP_LEFT[0]]

coefficients = np.array([
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [1, 1, 1, 1],
    [1, 0, 1, 0]
])

coefficients_inv = np.linalg.inv(coefficients)
a = np.dot(coefficients_inv, px)
b = np.dot(coefficients_inv, py)


# https://www.particleincell.com/2012/quad-interpolation/
def get_texture_coord(lat, lon):
    aa = a[3] * b[2] - a[2] * b[3]
    bb = a[3] * b[0] - a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + lon * b[3] - lat * a[3]
    cc = a[1] * b[0] - a[0] * b[1] + lon * b[1] - lat * a[1]

    det = math.sqrt(bb * bb - 4 * aa * cc)
    m = (-bb + det) / (2 * aa)

    l = (lon - a[0] - a[2] * m) / (a[1] + a[3] * m)

    return l, m


# https://vtk.org/Wiki/VTK/Examples/Cxx/Visualization/TextureMapPlane
def get_texture():
    reader = vtk.vtkJPEGReader()
    reader.SetFileName(TEXTURE_IMG)
    texture = vtk.vtkTexture()
    texture.SetInputConnection(reader.GetOutputPort())
    return texture


def generate_map():
    sgrid = vtk.vtkStructuredGrid()
    points = vtk.vtkPoints()
    coordinate_texture = vtk.vtkFloatArray()
    coordinate_texture.SetNumberOfComponents(2)
    scalars = vtk.vtkIntArray()

    data_map = np.fromfile(VTK_MAP, dtype=np.int16).reshape(MAP_WIDTH, MAP_WIDTH)
    delta_degree = DEGREE / MAP_WIDTH

    top = min(TOP_LEFT[0], TOP_RIGHT[0])
    bottom = max(BOTTOM_RIGHT[0], BOTTOM_LEFT[0])
    left = max(TOP_LEFT[1], BOTTOM_LEFT[1])
    right = min(TOP_RIGHT[1], BOTTOM_RIGHT[1])

    top_index = int((MAX_LAT - top) / delta_degree)
    bottom_index = int((MAX_LAT - bottom) / delta_degree)
    left_index = int((left - MIN_LONG) / delta_degree)
    right_index = int((right - MIN_LONG) / delta_degree)

    data_map = data_map[top_index:bottom_index, left_index:right_index]

    for i, row in enumerate(data_map):
        for j, altitude in enumerate(row):
            latitude = top - i * delta_degree
            longitude = left + j * delta_degree

            points.InsertNextPoint(coordinate_earth(latitude, longitude, altitude))

            l, m = get_texture_coord(latitude, longitude)
            coordinate_texture.InsertNextTuple((l, m))

            scalars.InsertNextValue(altitude)

    sgrid.SetPoints(points)

    dim_y, dim_x = data_map.shape
    sgrid.SetDimensions(dim_x, dim_y, 1)
    sgrid.GetPointData().SetTCoords(coordinate_texture)
    sgrid.GetPointData().SetScalars(scalars)

    # Mapper
    gridMapper = vtk.vtkDataSetMapper()
    gridMapper.SetInputData(sgrid)
    gridMapper.ScalarVisibilityOff()

    # Actor
    gridActor = vtk.vtkActor()
    gridActor.SetMapper(gridMapper)

    texture = get_texture()
    gridActor.SetTexture(texture)

    return gridActor


def generate_plane_path():
    size, coordinates = read_txt(VTK_PLANE_GPS)

    plane_points = vtk.vtkPoints()
    plane_lines = vtk.vtkPolyLine()
    plane_lines.GetPointIds().SetNumberOfIds(int(size))
    scalar = vtk.vtkFloatArray()

    previous_alt = coordinates[0][2]

    for i, (x, y, alt) in enumerate(coordinates):
        lat, long = convert_rt90_to_gps_coordinate(x, y)
        plane_points.InsertNextPoint(coordinate_earth(lat, long, alt))
        plane_lines.GetPointIds().SetId(i, i)

        delta_alt = previous_alt - alt
        scalar.InsertNextValue(delta_alt)
        previous_alt = alt

    min_scalar, max_scalar = scalar.GetValueRange()

    plane_cells = vtk.vtkCellArray()
    plane_cells.InsertNextCell(plane_lines)

    plane_data = vtk.vtkPolyData()
    plane_data.SetPoints(plane_points)
    plane_data.SetLines(plane_cells)
    plane_data.GetPointData().SetScalars(scalar)

    plane_tube = vtk.vtkTubeFilter()
    plane_tube.SetRadius(35)
    plane_tube.SetInputData(plane_data)

    plane_mapper = vtk.vtkPolyDataMapper()
    plane_mapper.SetInputConnection(plane_tube.GetOutputPort())
    plane_mapper.SetScalarRange(min_scalar, max_scalar)

    plane_actor = vtk.vtkActor()
    plane_actor.SetMapper(plane_mapper)

    return plane_actor


class MyInteractor(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, map_actor, text_actor):
        self.AddObserver("MouseMoveEvent", self.mouse_move_event)
        self.map_actor = map_actor
        self.text_actor = text_actor

        self.elevation_actor = vtk.vtkActor()
        self.sphere = vtk.vtkSphere()
        self.cutter = vtk.vtkCutter()
        self.stripper = vtk.vtkStripper()
        self. tube_filter = vtk.vtkTubeFilter()
        self.mapper = vtk.vtkDataSetMapper()
        self.picker = vtk.vtkPointPicker()

    def mouse_move_event(self, obj, event):
        pos = self.GetInteractor().GetEventPosition()
        self.picker.Pick(pos[0], pos[1], 0, self.GetDefaultRenderer())
        actor_picked = self.picker.GetActor()

        self.elevation_actor.GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d("midnight_blue"))
        self.GetDefaultRenderer().AddActor(self.elevation_actor)

        if actor_picked == self.map_actor:
            altitude = self.picker.GetDataSet().GetPointData().GetScalars().GetValue(self.picker.GetPointId())
            self.text_actor.SetInput(str(altitude) + "m")
            self.GetInteractor().Render()

            # We call the sphere (cutter/striper/tube_filter...)
            self._elevation_cut(altitude, self.picker.GetDataSet())

        self.OnMouseMove()
        return

    def _elevation_cut(self, altitude, picker_dataset):
        self.sphere.SetCenter(0, 0, 0)
        self.sphere.SetRadius(EARTH_RADIUS + altitude)

        self. cutter.SetCutFunction(self.sphere)
        self. cutter.SetInputData(picker_dataset)

        self.stripper.SetInputConnection(self.cutter.GetOutputPort())

        self.tube_filter.SetRadius(32)
        self.tube_filter.SetInputConnection(self.stripper.GetOutputPort())

        self.mapper.ScalarVisibilityOff()
        self.mapper.SetInputConnection(self.tube_filter.GetOutputPort())

        self.elevation_actor.SetMapper(self.mapper)


def main():
    # Map actor
    print("Genarate map actor...")
    map_actor = generate_map()

    # Actor plane
    print("Genarate plane actor...")
    plane_actor = generate_plane_path()

    # Text actor
    text_actor = vtk.vtkTextActor()
    text_actor.GetTextProperty().SetColor(1, 1, 1)
    text_actor.SetInput("0 m")

    # Render
    print('Setting the renderer')
    renderer = vtk.vtkRenderer()
    renderer.AddActor(map_actor)
    renderer.AddActor(plane_actor)
    renderer.AddActor(text_actor)
    renderer.SetBackground(0, 0, 0)

    # Custom interactor
    style = MyInteractor(map_actor, text_actor)
    style.SetDefaultRenderer(renderer)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)
    renWin.SetSize(WINDOW_WIDTH_SIZE, WINDOW_HEIGTH_SIZE)

    print("Finish")

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(style)

    text_widget = vtk.vtkTextWidget()
    text_widget.SetTextActor(text_actor)
    text_widget.SetInteractor(iren)
    text_widget.SelectableOff()
    text_widget.On()

    iren.Initialize()
    renWin.Render()
    iren.Start()


if __name__ == '__main__':

    main()
