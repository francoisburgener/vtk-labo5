"""
VTK - HEIG - Lab05
authors: François Burgener, Tiago Povoa Quinteiro
"""
import os
import time
import vtk
import numpy as np
import pyproj

# Constante
VTK_PLANE_GPS = "vtkgps.txt"
TEXTURE_IMG = "glider_map.jpg"
VTK_MAP = "EarthEnv-DEM90_N60E010.bil"
VTK_FILENAME = "plane.vtk"

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

# https://gis.stackexchange.com/questions/78838/converting-projected-coordinates-to-lat-lon-using-python
RT90 = pyproj.Proj(init='epsg:3857')
GPS = pyproj.Proj(init='epsg:4326')


def convert_rt90_to_gps_coordinate(x, y):
    longitude, latitude = pyproj.transform(RT90, GPS, x, y)
    return latitude, longitude


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


def generate_map(sgrid):
    data_map = np.fromfile(VTK_MAP, dtype=np.int16).reshape(MAP_WIDTH, MAP_WIDTH)

    delta = DEGREE / MAP_WIDTH

    

    points = vtk.vtkPoints()

    # exploring the values
    for i, row in enumerate(data_map):
        for j, alt in enumerate(row):
            print("TODO")

    return data_map


def main():
    sgrid = vtk.vtkStructuredGrid()
    data_map = generate_map(sgrid)


main()
