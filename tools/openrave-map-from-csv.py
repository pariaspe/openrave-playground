#!/usr/bin/env python

# Authors: Juan G Victores
# CopyPolicy: released under the terms of the LGPLv2.1
# URL: https://github.com/roboticslab-uc3m/openrave-tools

from lxml import etree

#-- User variables
boxHeight = 1.0
inFileStr = 'assets/map1.csv'

resolution = 1.0  # Just to make similar to MATLAB [pixel/meter]
meterPerPixel = 1 / resolution  # [meter/pixel]

#-- Program
from numpy import genfromtxt
inFile = genfromtxt(inFileStr, delimiter=',')
print(inFile)

nX = inFile.shape[0]
nY = inFile.shape[1]
print("lines = X =",inFile.shape[0])
print("columns = Y =",inFile.shape[1])

#-- Default to X=rows,Y=columns. Uncomment the next 3 lines to transpose.
# print("transposing")
# from numpy import transpose
# inFile = transpose(inFile)

Ez = boxHeight / 2.0  # Box size is actually double the extent

Ex = meterPerPixel / 2.0
Ey = meterPerPixel / 2.0

KinBody = etree.Element("KinBody", name="map")

#-- Create Floor
floorEx = Ex * nX
floorEy = Ey * nY
floorEz = boxHeight / 8.0  # arbitrary
Body = etree.SubElement(KinBody, "Body", name="floor", type="static")
Geom = etree.SubElement(Body, "Geom", type="box")
Extents = etree.SubElement(Geom, "Extents").text= str(floorEx)+" "+ str(floorEy)+" "+str(floorEz)
Translation = etree.SubElement(Geom, "Translation").text= str(floorEx)+" "+str(floorEy)+" "+str(-floorEz)
DifusseColor = etree.SubElement(Geom, "diffuseColor").text= ".1 .1 .1"

#-- Create Walls
for iX in range(nX):
    #print("iX:",iX)
    for iY in range(nY):
        #print("* iY:",iY)

        #-- Skip box if map indicates a 0
        if inFile[iX][iY] == 0:
            continue

        #-- Add E___ to each to force begin at 0,0,0 (centered by default)
        x = Ex + iX*meterPerPixel
        y = Ey + iY*meterPerPixel
        z = Ez  # Add this to raise to floor level (centered by default)

        #-- Create box
        Body = etree.SubElement(KinBody, "Body", name="box_"+str(iX)+"_"+str(iY), type="static")
        Geom = etree.SubElement(Body, "Geom", type="box")
        Extents = etree.SubElement(Geom, "Extents").text= str(Ex)+" "+ str(Ey)+" "+str(Ez)
        Translation = etree.SubElement(Geom, "Translation").text= str(x)+" "+str(y)+" "+str(z)
        DifusseColor = etree.SubElement(Geom, "diffuseColor").text= ".5 .5 .5"

myStr = etree.tostring(KinBody, pretty_print=True, encoding="unicode")

outFile = open('map.kinbody.xml', 'w')
outFile.write(myStr)
outFile.close()
