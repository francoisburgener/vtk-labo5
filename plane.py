"""
VTK - HEIG - Lab05
authors: François Burgener, Tiago Povoa Quinteiro
"""
import os
import time
import vtk
import numpy as np
from pyproj import Transformer

# Constante file
VTK_PLANE_GPS = "vtkgps.txt"
TEXTURE_IMG = "glider_map.jpg"
VTK_MAP = "EarthEnv-DEM90_N60E010.bil"
VTK_FILENAME = "plane.vtk"


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


RT90 = "epsg:3857"
GPS = "epsg:4326"

#https://pyproj4.github.io/pyproj/stable/gotchas.html#upgrading-to-pyproj-2-from-pyproj-1
def convert_rt90_to_gps_coordinate(x, y):
    transformer = Transformer.from_crs(RT90, GPS)
    return transformer.transform(x, y)


TOP_LEFT = convert_rt90_to_gps_coordinate(1349340, 7022573)
TOP_RIGHT = convert_rt90_to_gps_coordinate(1371573, 7022967)
BOTTOM_LEFT = convert_rt90_to_gps_coordinate(1349602, 7005969)
BOTTOM_RIGHT = convert_rt90_to_gps_coordinate(1371835, 7006362)

def writer_vtk(filename, data):
    """
    Writes a structured grid reader into a file
    :param filename: the name of the file
    :param data: An structuredGrid in our case, but it could be more generic
    :return: void
    """
    writer = vtk.vtkDataSetWriter()
    writer.SetFileName(filename)
    writer.SetInputData(data)
    writer.Write()


def reader_vtk(filename):
    """
    Reads a structured grid reader from a file
    :param filename: the name of the file
    :return: A structured grid reader
    """
    reader = vtk.vtkStructuredGridReader()
    reader.SetFileName(filename)
    reader.Update()
    return reader


def read_txt(filename):
    """
    Reads the raw output data from a file.
    :param filename: the file name
    :return: TODO
    """
    with open(filename, 'r', encoding="utf-8") as fd:
        size = fd.readline()
        lines = fd.readlines()

        coordinate = []

        for line in lines:
            tmp = line.split(' ')
            x = int(tmp[1])
            y = int(tmp[2])
            z = int(tmp[3])

            coordinate.append((x, y, z))

        return size, coordinate


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
    transform.RotateX(lat)
    transform.Translate(0, 0, EARTH_RADIUS + alt)

    return transform.TransformPoint(0, 0, 0)


def generate_plane(coordinate):
    return 0

# https://vtk.org/Wiki/VTK/Examples/Cxx/Visualization/TextureMapPlane
def get_texture():
    reader = vtk.vtkJPEGReader()
    reader.SetFileName(TEXTURE_IMG)
    texture = vtk.vtkTexture()
    texture.SetInputConnection(reader.GetOutputPort())
    return texture

def generate_map(sgrid):
    data_map = np.fromfile(VTK_MAP, dtype=np.int16).reshape(MAP_WIDTH, MAP_WIDTH)

    delta_long = (MAX_LONG - MIN_LONG) / MAP_WIDTH
    delta_lat = (MAX_LAT - MIN_LAT) / MAP_WIDTH
    points = vtk.vtkPoints()
    altitude_values = vtk.vtkIntArray()

    for i, row in enumerate(data_map):
        for j, altitude in enumerate(row):

            # Calcul of latitude, longitude for each point
            latitude = MIN_LAT + i * delta_lat
            longitude = MIN_LONG + j * delta_long

            points.InsertNextPoint(coordinate_earth(latitude, longitude, altitude))
            altitude_values.InsertNextValue(altitude)

    sgrid.SetPoints(points)
    sgrid.SetDimensions([MAP_WIDTH, MAP_WIDTH, 1])
    sgrid.GetPointData().SetScalars(altitude_values)


def main():
    sgrid = vtk.vtkStructuredGrid()

    """
    If we exec the program for the first time, we have to run some calculations
    Otherwise we'll just read the file. So we have a sort of cache to speed up
    """
    if not os.path.isfile(VTK_FILENAME):
        print('Initial read from data')
        generate_map(sgrid)
        writer_vtk(VTK_FILENAME, sgrid)

    print('Read from vtk file')
    reader = reader_vtk(VTK_FILENAME)

    # Mapper
    print('Setting the Mapper')
    gridMapper = vtk.vtkDataSetMapper()
    gridMapper.SetInputData(reader.GetOutput())

    # Actor
    print('Setting the Actor')
    gridActor = vtk.vtkActor()
    gridActor.SetMapper(gridMapper)

    # Render
    print('Setting the renderer')
    renderer = vtk.vtkRenderer()
    renderer.AddActor(gridActor)
    renderer.SetBackground(0.5, 0.5, 0.5)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(renderer)
    renWin.SetSize(600, 860)
    renWin.Render()

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())

    # Generates the .PNG file
    # wif = vtk.vtkWindowToImageFilter()
    # wif.SetInput(renWin)
    # wif.Update()
    #
    # writer = vtk.vtkPNGWriter()
    # writer.SetFileName("map_{}.png".format(SEA_ALT))
    # writer.SetInputConnection(wif.GetOutputPort())
    # writer.Write()

    print("Finish")

    # Interact with the data.
    iren.Initialize()
    iren.Start()


main()
