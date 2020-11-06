def filterTypedef(typedef):
  # error: ?
  if (
    typedef.spelling == "Handle_Font_BRepFont" or
    typedef.spelling == "Handle_PCDM_Reader" or
    typedef.spelling == "Handle_PCDM_ReadWriter_1"
  ):
    return False

  if typedef.spelling == "OpenGl_ListOfStructure":
    return False

  # Generates error "Cannot register type 'TColQuantity_Array1OfLength' twice" during initialization of the WASM file. Seems to be conflicting with 'TColStd_Array1OfReal'. Can be reproduced by including these two bindings in one bindings-file.
  if typedef.spelling == "TColQuantity_Array1OfLength":
    return False
  # Same as above, but with TopoDS_ListOfShape / TopTools_ListOfShape
  if typedef.spelling == "TopoDS_ListOfShape":
    return False
  # Same as above, but with Handle_Graphic3d_Structure / Handle_Prs3d_Presentation
  if typedef.spelling == "Handle_Graphic3d_Structure":
    return False
  # Cannot register type 'PCDM_BaseDriverPointer' twice
  if typedef.spelling == "PCDM_BaseDriverPointer":
    return False

  # error: unknown type name 'Handle_Xw_Window'; did you mean 'Handle_Cocoa_Window'?
  if typedef.spelling == "Handle_Xw_Window":
    return False

  # error: member pointer refers into non-class type 'opencascade::handle<TCollection_HAsciiString> (*)(const opencascade::handle<MoniTool_TypedValue> &, const opencascade::handle<TCollection_HAsciiString> &, bool)'
  # error: 'MoniTool_ValueInterpret' (aka 'handle<TCollection_HAsciiString> (*)(const handle<MoniTool_TypedValue> &, const handle<TCollection_HAsciiString> &, const bool)') is not a class, namespace, or enumeration
  if typedef.spelling == "MoniTool_ValueInterpret":
    return False

  # error: member pointer refers into non-class type 'opencascade::handle<TCollection_HAsciiString> (*)(const opencascade::handle<Interface_TypedValue> &, const opencascade::handle<TCollection_HAsciiString> &, bool)'
  # error: 'Interface_ValueInterpret' (aka 'handle<TCollection_HAsciiString> (*)(const handle<Interface_TypedValue> &, const handle<TCollection_HAsciiString> &, const bool)') is not a class, namespace, or enumeration
  if typedef.spelling == "Interface_ValueInterpret":
    return False

  return True