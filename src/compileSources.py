#!/usr/bin/python3

import os
import subprocess
import multiprocessing

from filter.filterIncludeFiles import filterIncludeFile
from filter.filterSourceFiles import filterSourceFile
from filter.filterPackagesAndModules import filterPackagesAndModules

libraryBasePath = "/opencascade.js/build/sources"

try:
  os.makedirs(libraryBasePath)
except:
  pass

# Potentially problematic packages, when used with dynamic linking
# These files contain function pointer definitions and header files and are therefore likely to cause problems.
# https://github.com/emscripten-core/emscripten/issues/13241
# "AdvApp2Var"
# "BRepGProp"
# "BRepMesh"
# "BSplSLib"
# "CPnts"
# "DDF"
# "Draw"
# "Graphic3d"
# "IFSelect"
# "Interface"
# "MoniTool"
# "NCollection"
# "OpenGl"
# "OSD"
# "ShapeProcess"
# "Standard"
# "StdObjMgt"
# "TDF

includePaths = []
includePaths.extend([
  "/rapidjson/include",
  "/freetype/include/freetype",
  "/freetype/include",
])
for dirpath, dirnames, filenames in os.walk(os.path.join("/occt/occt-" + os.environ['OCCT_COMMIT_HASH'] + "/src/")):
  includePaths.append(dirpath)

def buildObjectFiles(package, release = True, debug = True):
  packageName = package["name"]
  packageFiles = package["files"]
  try:
    os.makedirs(libraryBasePath + "/" + packageName)
  except:
    pass
  for fileToBuild in packageFiles:
    theFile = fileToBuild["file"]
    theName = fileToBuild["name"]
    dirpath = fileToBuild["dirpath"]
    libBasePath = libraryBasePath + "/" + packageName + "/" + theName
    command = [
      "emcc",
      "-DIGNORE_NO_ATOMICS=1", "-DOCCT_NO_PLUGINS", "-frtti", "-fPIC", "-DHAVE_RAPIDJSON",
      *list(map(lambda x: "-I" + x, includePaths)),
      "-c",
      dirpath + "/" + theFile,
    ]

    if release:
      if not os.path.exists(libBasePath + ".o"):
        print("Building " + theName + ".o")
        subprocess.check_call([
          *command,
          "-O3",
          "-o", libBasePath + ".o",
          ])
      else:
        print(theName + ".o already exists, skipping")

    if debug:
      if not os.path.exists(libBasePath + ".debug.o"):
        print("Building " + theName + ".debug.o")
        subprocess.check_call([
          *command,
          "-gsource-map",
          "-o", libBasePath + ".debug.o",
        ])
      else:
        print(theName + ".debug.o already exists, skipping")

objectFilesToBuild = []

def addObjectFiles(packageName):
  thisPackage = {}
  thisPackage["name"] = packageName
  thisPackage["files"] = []
  objectFilesToBuild.append(thisPackage)
  for dirpath, dirnames, filenames in os.walk(os.path.join("/occt/occt-" + os.environ['OCCT_COMMIT_HASH'] + "/src/", packageName)):
    for item in filenames:
      if filterSourceFile(item):
        thisPackage["files"].append({
          "name": os.path.splitext(item)[0],
          "file": item,
          "dirpath": dirpath
        })

for dirpath, dirnames, filenames in os.walk("/occt/occt-" + os.environ['OCCT_COMMIT_HASH'] + "/src/"):
  packageOrModuleName = os.path.basename(dirpath)
  if not any(x for x in filenames if x == "PACKAGES"):
    continue

  if not filterPackagesAndModules(packageOrModuleName):
    continue

  with open(dirpath + "/PACKAGES", "r") as a_file:
    for package in a_file:
      packageName = package.strip()
      if not filterPackagesAndModules(packageName):
        continue
      addObjectFiles(packageName)

def myBuildObjectFiles(x):
  buildObjectFiles(x, True, False)

with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as p:
  p.map(myBuildObjectFiles, objectFilesToBuild)