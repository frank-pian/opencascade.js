#include <vector>
#include <TopoDS.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepTools.hxx>
#include <Poly.hxx>
#include <STEPControl_Writer.hxx>
#include <StepData_StepWriter.hxx>
#include <StepData_StepModel.hxx>
#include <StepData_Protocol.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <Poly_Triangulation.hxx>
#include <StlAPI_Writer.hxx>
#include <BRepExtrema_ShapeProximity.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFApp_Application.hxx>
#include <RWObj_CafWriter.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <Message_ProgressRange.hxx>
#include <APIHeaderSection_MakeHeader.hxx>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#define EMBIND_LAMBDA(retval, arglist, impl) (retval (*) arglist) [] arglist impl

namespace ImportData
{
    class TriangleData
    {
    public:
        std::vector<float> vertexs;
        std::vector<float> normals;
        std::vector<float> uvs;
        std::vector<int> indices;
    };

    TriangleData *getTriangleData(TopoDS_Shape topoShape, double theLinDeflection, double theAngDeflection)
	{
		BRepMesh_IncrementalMesh(topoShape, theLinDeflection, false, theAngDeflection);
		TriangleData *triData = new TriangleData();
		int newStart = 0;
		for (TopExp_Explorer aFaceExplorer(topoShape, TopAbs_FACE); aFaceExplorer.More(); aFaceExplorer.Next())
		{
			TopoDS_Face aFace = TopoDS::Face(aFaceExplorer.Current());
			TopLoc_Location location;
			Handle(Poly_Triangulation) triFace = BRep_Tool::Triangulation(aFace, location);
			if (triFace.IsNull())
				continue;
			int nbTri = triFace->NbTriangles();
			Poly::ComputeNormals(triFace);
			auto nodes = triFace->InternalNodes();
			auto UVNodes = triFace->InternalUVNodes();
			auto Normals = triFace->InternalNormals();
			auto tris = triFace->InternalTriangles();
			for (Standard_Integer aNodeIter = nodes.Lower(); aNodeIter <= nodes.Upper(); ++aNodeIter)
			{
				gp_Pnt vertex = nodes.Value(aNodeIter).Transformed(location.Transformation());
				gp_Pnt2d uv = UVNodes.Value(aNodeIter);
				gp_Vec3f normalf = Normals.Value(aNodeIter);
				gp_Dir normal = gp_Dir(normalf.x(), normalf.y(), normalf.z());
				triData->vertexs.push_back(vertex.X());
				triData->vertexs.push_back(vertex.Y());
				triData->vertexs.push_back(vertex.Z());
				triData->normals.push_back(normal.X());
				triData->normals.push_back(normal.Y());
				triData->normals.push_back(normal.Z());
				triData->uvs.push_back(uv.X());
				triData->uvs.push_back(uv.Y());
			}
			int max = 0;
			for (int i = 1; i <= nbTri; i++)
			{
				Poly_Triangle aTriangle = triFace->Triangle(i);
				int Index1 = 0;
				int Index2 = 0;
				int Index3 = 0;
				if ((aFace.Orientation() == TopAbs_REVERSED))
				{
					Index1 = aTriangle.Value(1) - 1;
					Index2 = aTriangle.Value(3) - 1;
					Index3 = aTriangle.Value(2) - 1;
				}
				else
				{
					Index1 = aTriangle.Value(1) - 1;
					Index2 = aTriangle.Value(2) - 1;
					Index3 = aTriangle.Value(3) - 1;
				}
				if (Index1 > max)
				{
					max = Index1;
				}
				if (Index2 > max)
				{
					max = Index2;
				}
				if (Index3 > max)
				{
					max = Index3;
				}
				triData->indices.push_back(Index1 + newStart);
				triData->indices.push_back(Index2 + newStart);
				triData->indices.push_back(Index3 + newStart);
			}
			newStart += max + 1;
		}
		return triData;
	}

	emscripten::val importStep(TopoDS_Shape topoShape) {
		std::ostringstream fileStream;
		STEPControl_Writer writer;
		writer.Transfer(topoShape, STEPControl_AsIs);
		APIHeaderSection_MakeHeader makeHeader(writer.Model());
		Handle(TCollection_HAsciiString) fileDescription = new TCollection_HAsciiString("Designed by EasyEDA Pro");
		makeHeader.SetDescriptionValue(1, fileDescription);
		Handle_StepData_StepModel stepModel = writer.Model();
		StepData_StepWriter dumper(stepModel);
		Handle_StepData_Protocol protocol = Handle_StepData_Protocol::DownCast(stepModel->Protocol());
		dumper.SendModel(protocol);
		dumper.Print(fileStream);

		char *byteBuffer = strdup(fileStream.str().c_str());
		size_t bufferLength = strlen(byteBuffer);
		return emscripten::val(emscripten::typed_memory_view(bufferLength, byteBuffer));
	}

	bool shapeProximity(TopoDS_Shape topoShape1, TopoDS_Shape topoShape2) {
		BRepExtrema_ShapeProximity aTool;
		aTool.LoadShape1(topoShape1);
		aTool.LoadShape2(topoShape2);
		aTool.Perform();
		return aTool.IsDone();
	}

	emscripten::val importStl(TopoDS_Shape topoShape, Standard_Real aLinearDeflection) {
		StlAPI_Writer aWriter;
		const char* filename = "tempstl";

		BRepMesh_IncrementalMesh mesh(topoShape, aLinearDeflection);
		mesh.Perform();

		aWriter.ASCIIMode() = Standard_True;
		aWriter.Write(topoShape, filename);
		
		std::ifstream ifs(filename, std::ios::binary);
		ifs.seekg(0, std::ios::end);
		size_t fileSize = ifs.tellg();
		ifs.seekg(0, std::ios::beg);
		std::string buffer(fileSize, ' ');
		ifs.read(&buffer[0], fileSize);
		size_t pos = buffer.find("solid");
		if (pos != std::string::npos) {
			buffer.replace(pos, 5, "solid Designed by EasyEDA Pro");
		}
		char *byteBuffer = strdup(buffer.c_str());
		size_t bufferLength = strlen(byteBuffer);
		return emscripten::val(emscripten::typed_memory_view(bufferLength, byteBuffer));
	}

	emscripten::val importObj(TopoDS_Shape topoShape, Standard_Real aLinearDeflection) {
		TColStd_IndexedDataMapOfStringString aFileInfo;
		aFileInfo.Add("Author", "Easy EDA");
		TCollection_AsciiString tempFile("tmpObj");

		BRepMesh_IncrementalMesh mesh(topoShape, aLinearDeflection);
		mesh.Perform();

		Handle(TDocStd_Document) aDoc;
  		Handle(TDocStd_Application) anApp = XCAFApp_Application::GetApplication();
		anApp->NewDocument (TCollection_ExtendedString ("BinXCAF"), aDoc);
        Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool (aDoc->Main());
        aShapeTool->AddShape (topoShape);

		Standard_Real aFileUnitFactor = 1.0;
		const Standard_Real aSystemUnitFactor = UnitsMethods::GetCasCadeLengthUnit();
  		RWMesh_CoordinateSystem aSystemCoordSys = RWMesh_CoordinateSystem_Zup, aFileCoordSys = RWMesh_CoordinateSystem_Yup;

		RWObj_CafWriter aWriter(tempFile);
		aWriter.ChangeCoordinateSystemConverter().SetInputLengthUnit (aSystemUnitFactor);
		aWriter.ChangeCoordinateSystemConverter().SetInputCoordinateSystem (aSystemCoordSys);
		aWriter.ChangeCoordinateSystemConverter().SetOutputLengthUnit (aFileUnitFactor);
		aWriter.ChangeCoordinateSystemConverter().SetOutputCoordinateSystem (aFileCoordSys);
		Message_ProgressRange pi;
		aWriter.Perform (aDoc, aFileInfo, pi);

		std::string fileName = "tmpObj";
    	std::ifstream inFile(fileName.c_str());
		inFile.is_open();
		inFile.seekg(0, std::ios::end);
    	int length = inFile.tellg();
    	inFile.seekg(0, std::ios::beg);
		char* buffer = new char[length];
		inFile.read(buffer, length);
		inFile.close();
		size_t bufferLength = strlen(buffer);
		return emscripten::val(emscripten::typed_memory_view(bufferLength, buffer));
	}
}

EMSCRIPTEN_BINDINGS(ImportData) {
    emscripten::class_<ImportData::TriangleData>("TriangleData")
		.function("vertexs", EMBIND_LAMBDA(emscripten::val, (ImportData::TriangleData* triData), {
			 std::vector<float> c_datas = triData->vertexs;
	         emscripten::val js_Datas = emscripten::val::array();
			 for (int i = 0; i < c_datas.size(); i++)
			 {
				 js_Datas.set(i, c_datas[i]);
			 }
			 return js_Datas;
		}), emscripten::allow_raw_pointers())
		.function("normals", EMBIND_LAMBDA(emscripten::val, (ImportData::TriangleData* triData), {
			 std::vector<float> c_datas = triData->normals;
	         emscripten::val js_Datas = emscripten::val::array();
			 for (int i = 0; i < c_datas.size(); i++)
			 {
				 js_Datas.set(i, c_datas[i]);
			 }
			 return js_Datas;
		}), emscripten::allow_raw_pointers())
		.function("uvs", EMBIND_LAMBDA(emscripten::val, (ImportData::TriangleData* triData), {
			 std::vector<float> c_datas = triData->uvs;
	         emscripten::val js_Datas = emscripten::val::array();
			 for (int i = 0; i < c_datas.size(); i++)
			 {
				 js_Datas.set(i, c_datas[i]);
			 }
			 return js_Datas;
		}), emscripten::allow_raw_pointers())
		.function("indices", EMBIND_LAMBDA(emscripten::val, (ImportData::TriangleData* triData), {
			 std::vector<int> c_datas = triData->indices;
	         emscripten::val js_Datas = emscripten::val::array();
			 for (int i = 0; i < c_datas.size(); i++)
			 {
				 js_Datas.set(i, c_datas[i]);
			 }
			 return js_Datas;
		}), emscripten::allow_raw_pointers());
	
	emscripten::function("getTriangleData", &ImportData::getTriangleData, emscripten::allow_raw_pointers());

	emscripten::function("importStep", &ImportData::importStep, emscripten::allow_raw_pointers());

	emscripten::function("shapeProximity", &ImportData::shapeProximity);

	emscripten::function("importStl", &ImportData::importStl, emscripten::allow_raw_pointers());

	emscripten::function("importObj", &ImportData::importObj, emscripten::allow_raw_pointers());
}
