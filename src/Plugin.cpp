#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>

#include <xsi_icenodecontext.h>
#include <xsi_icenodedef.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_longarray.h>
#include <xsi_doublearray.h>
#include <xsi_math.h>
#include <xsi_vector2f.h>
#include <xsi_vector3f.h>
#include <xsi_vector4f.h>
#include <xsi_matrix3f.h>
#include <xsi_matrix4f.h>
#include <xsi_rotationf.h>
#include <xsi_quaternionf.h>
#include <xsi_color4f.h>
#include <xsi_shape.h>
#include <xsi_icegeometry.h>
#include <xsi_iceportstate.h>
#include <xsi_indexset.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>

#include "ConvexHull.h"

// Defines port, group and map identifiers used for registering the ICENode
enum IDs
{
	ID_IN_Positions = 1,
	ID_IN_Triangulate,
	ID_G_100 = 100,
	ID_OUT_PolygonalDescription = 200,
	ID_TYPE_CNS = 400,
	ID_STRUCT_CNS,
	ID_CTXT_CNS,
	ID_UNDEF = ULONG_MAX
};

XSI::CStatus RegisterConvexHull( XSI::PluginRegistrar& in_reg );

using namespace XSI; 

SICALLBACK XSILoadPlugin( PluginRegistrar& in_reg )
{
	in_reg.PutAuthor(L"Guillaume Laforge");
	in_reg.PutName(L"Convex Hull Plugin");
	in_reg.PutVersion(1,0);

	RegisterConvexHull( in_reg );

	//RegistrationInsertionPoint - do not remove this line

	return CStatus::OK;
}

SICALLBACK XSIUnloadPlugin( const PluginRegistrar& in_reg )
{
	CString strPluginName;
	strPluginName = in_reg.GetName();
	Application().LogMessage(strPluginName + L" has been unloaded.",siVerboseMsg);
	return CStatus::OK;
}

CStatus RegisterConvexHull( PluginRegistrar& in_reg )
{
	ICENodeDef nodeDef;
	nodeDef = Application().GetFactory().CreateICENodeDef(L"ConvexHull",L"Convex Hull");

	CStatus st;
	st = nodeDef.PutColor(154,188,102);
	st.AssertSucceeded( ) ;

	st = nodeDef.PutThreadingModel(XSI::siICENodeSingleThreading);
	st.AssertSucceeded( ) ;

	// Add input ports and groups.
	st = nodeDef.AddPortGroup(ID_G_100);
	st.AssertSucceeded( ) ;

	st = nodeDef.AddInputPort(ID_IN_Positions,ID_G_100,siICENodeDataVector3,siICENodeStructureAny, siICENodeContextAny, L"Positions",L"Positions",CValue(),CValue(),CValue(),ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;
	
	st = nodeDef.AddInputPort(ID_IN_Triangulate,ID_G_100,siICENodeDataBool,siICENodeStructureSingle, siICENodeContextSingleton, L"Triangulate Coplanar",L"triangulate", false,CValue(),CValue(),ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;

	// Add output ports.
	st = nodeDef.AddOutputPort(ID_OUT_PolygonalDescription,siICENodeDataLong,siICENodeStructureArray,siICENodeContextSingleton,L"Polygonal Description",L"polygonaldescription",ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;

	PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
	nodeItem.PutCategories(L"Topology");

	return CStatus::OK;
}

SICALLBACK ConvexHull_Evaluate( ICENodeContext& in_ctxt )
{
	// Get input data
	siICENodeDataType inPortType;
	siICENodeStructureType inPortStruct;
	siICENodeContextType inPortContext;
			
	in_ctxt.GetPortInfo( ID_IN_Positions, inPortType, inPortStruct, inPortContext );
			

	CDataArrayBool inTriangulate( in_ctxt, ID_IN_Triangulate);
	bool isTriangulate = inTriangulate[0];

	ConvexHull convexHull;
			
	if( inPortStruct == siICENodeStructureArray && inPortContext == siICENodeContextSingleton )
	{
		CDataArray2DVector3f inPositions( in_ctxt, ID_IN_Positions);
		convexHull.GetPointsSet(inPositions, isTriangulate);
	}
	else if( inPortStruct == siICENodeStructureSingle  && inPortContext != siICENodeContextSingleton )
	{
		CIndexSet indexSet( in_ctxt, ID_IN_Positions );
		CDataArrayVector3f inPositions( in_ctxt, ID_IN_Positions);
		convexHull.GetPointsSet(indexSet, inPositions, isTriangulate);
	}
			
	if(convexHull.IsValid())
	{
					
		LongArray l_polyDescription = convexHull.BuildHull(isTriangulate);

		// Get output data
		CDataArray2DLong outData( in_ctxt );
		CDataArray2DLong::Accessor outAccessor;							
		outAccessor = outData.Resize(0, static_cast<ULONG>(l_polyDescription.size()));

		for(ULONG i=0; i<l_polyDescription.size(); i++)
		{
			outAccessor[i] = l_polyDescription[i];
		}
	}			
	
	return CStatus::OK;
}

