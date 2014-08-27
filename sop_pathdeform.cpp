/*
 * sop_pathdeform.cpp
 *
 *  Created on: Aug 10, 2014
 *      Author: Alexey Rusev(Sadrutdinov)
 *      e-mail: hou.alex@gmail.com
 */

#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GU/GU_Prim.h>
#include <UT/UT_Vector.h>
#include <UT/UT_Array.h>
#include <GA/GA_PrimitiveFamilyMask.h>
#include <GA/GA_AttributeRefMapDestHandle.h>
#include <GA/GA_Types.h>
#include <OP/OP_OperatorTable.h>
#include <GEO/GEO_Primitive.h>
#include <GEO/GEO_PrimTypeCompat.h>
#include <GEO/GEO_PrimType.h>
#include <GEO/GEO_Curve.h>
#include <GOP/GOP_AttribListParse.h>
#include <GU/GU_Curve.h>
#include <PRM/PRM_Include.h>
#include <SYS/SYS_Math.h>
#include "sop_pathdeform.h"


using namespace std;

const char *inputLabels[] = {"Deform geometry", "Input curve"};

OP_Node *PathDeform::MyConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
	return new PathDeform(net, name, op);
}

PathDeform::PathDeform(OP_Network *net, const char *name, OP_Operator *op)
:SOP_Node(net, name, op)
{
}

PathDeform::~PathDeform() {};

static PRM_Name useUpVector("use_up_vector", "Use Up-Vector");
static PRM_Name useCurveTwist("use_curve_twist", "Use Twist Attribute");
static PRM_Name useCurveWidth("use_curve_width", "Scale By Width Attribute");
static PRM_Name stretch("stretch", "Stretch");
static PRM_Name stretch_tolen("stretch_to_len", "Stretch To Length");
static PRM_Name vecAttribs("vattribs", "Vector Attributes");
static PRM_Name transformVattribs("transform_vattribs", "Transform Vector Attributes");
static PRM_Name recompute_normals("recompute_n", "Recompute Point Normals");

static PRM_Range stretchRange(PRM_RANGE_RESTRICTED, -1, PRM_RANGE_UI, 2);

PRM_Template
PathDeform::parmsTemplatesList[] =
{
	PRM_Template(PRM_TOGGLE_E, 1, &useUpVector, PRMzeroDefaults),
	PRM_Template(PRM_XYZ, 3, &PRMupVectorName, PRMyaxisDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &useCurveTwist, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &useCurveWidth, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &recompute_normals, PRMzeroDefaults),
    PRM_Template(PRM_TOGGLE_E, 1, &stretch_tolen, PRMzeroDefaults),
    PRM_Template(PRM_FLT_J, 1, &stretch, PRMzeroDefaults, 0, &stretchRange),
	PRM_Template(PRM_TOGGLE_E, 1, &transformVattribs, PRMzeroDefaults),
    PRM_Template(PRM_STRING, 1, &vecAttribs, 0),
	PRM_Template(PRM_FLT_J, 1, &PRMoffsetName, PRMzeroDefaults),
	PRM_Template(),
};

bool
PathDeform::updateParmsFlags()
{
	bool changes = false;
//	changes |= enableParm(PRMupVectorName.getToken(), !PARM_USENORMALS());
	changes |= setVisibleState(PRMupVectorName.getToken(), PARM_USEUPVECTOR());
    changes |= enableParm(stretch.getToken(), !PARM_STRETCH_TOLEN());
    changes |= enableParm(vecAttribs.getToken(), PARM_TRANSFORM_VECTORS());
	return changes;
}


float
pointRelativeToBbox(UT_BoundingBox &bbox, const UT_Vector3 &pt, const int &axis = 2)
{
	UT_Vector3 min = bbox.minvec();
	UT_Vector3 max = bbox.maxvec();
	return SYSfit(pt[axis], min[axis], max[axis], 0, 1);
}

double
lerp(double &a, double &b, float weight)
{
	return a + weight * (b - a);
}


UT_Vector3D
lerp(UT_Vector3D &a, UT_Vector3D &b, float weight)
{
	double x,y,z;
	x = lerp(a[0], b[0], weight);
	y = lerp(a[1], b[1], weight);
	z = lerp(a[2], b[2], weight);
	return UT_Vector3D(x, y, z);
}

void
computeBboxAxis(UT_BoundingBox &bbox, UT_Vector3 &pt0, UT_Vector3 &pt1)
{
	UT_Vector3 min, max;
	min = bbox.minvec();
	max = bbox.maxvec();
	float x = min[0] + max[0];
	float y = min[1] + max[1];

	pt0.assign(x, y, min[2]);
	pt1.assign(x, y, max[2]);

}

void
PathDeform::computeCurveAttributes(const GEO_Curve *curve_prim, fpreal time)
{
	int use_curve_twist = PARM_USETWIST();
	UT_Vector3 prevP, curP, nextP, tang, btang, up, avg_normal;

	if (PARM_USEUPVECTOR())
		avg_normal.assign(PARM_UPX(time), PARM_UPY(time), PARM_UPZ(time));
	else
		((GU_Curve *)curve_prim->castTo())->curveNormal(avg_normal);

	avg_normal.normalize();
	short int npts = curve_prim->getPointRange().getEntries();
	for(GA_Iterator it(curve_prim->getPointRange()); !it.atEnd(); it.advance())
	{
		GA_Offset ptof = it.getOffset();
        curP = hndl_curve_p.get(ptof);
        if (ptof == curve_prim->getPointOffset(0)) // first point
        {
			nextP = hndl_curve_p.get(ptof + 1);
        	tang = curP - nextP;

		}
		else if(ptof == curve_prim->getPointOffset(npts - 1)) // last point
		{
			nextP = hndl_curve_p.get(ptof - 1);
			tang = nextP - curP;
		}
		else
		{
			prevP = hndl_curve_p.get(ptof - 1);
			nextP = hndl_curve_p.get(ptof + 1);
        	tang = prevP - nextP;
		}
        tang.normalize();
        btang = cross(tang, avg_normal);
        btang.normalize();
		up = cross(btang, tang);

		if (use_curve_twist && hndl_curve_twist.isValid())
		{
			UT_QuaternionD quat(SYSdegToRad(hndl_curve_twist.get(ptof)), tang);
			btang = quat.rotate(btang);
			up = quat.rotate(up);
		}

		hndl_curve_tang.set(ptof, tang);
		hndl_curve_btang.set(ptof, btang);
		hndl_curve_up.set(ptof, up);

	}
}

OP_ERROR
PathDeform::cookMySop(OP_Context &context)
{
	if (lockInput(0, context) >= UT_ERROR_ABORT)
	{
		return error();
	}

	if (lockInput(1, context) >= UT_ERROR_ABORT)
	{
		addError(SOP_ERR_INVALID_SRC, "No second input");
		return error();
	}

	duplicateSource(0, context);
	fpreal time = context.getTime();
	GU_Detail *curve_gdp = new GU_Detail(inputGeo(1, context));

	GEO_Primitive *curve_geo_prim = curve_gdp->getGEOPrimitive(curve_gdp->primitiveOffset(0));
	if (!curve_geo_prim)
	{
		addError(OP_ERR_INVALID_SRC, "Can't find curve primitive");
        return error();
	}
	else if (curve_geo_prim->getTypeDef().getFamilyMask() != GA_FAMILY_FACE)
	{

		addError(OP_ERR_INVALID_SRC, "Primitive is not a polycurve type");
		return error();
	}

	// Parms
	int use_width = PARM_USEWIDTH();
    int stretch_tolen = PARM_STRETCH_TOLEN();
    int recompute_n = PARM_COMPUTE_N();
	// Curve attributes
	aref_curve_tang = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "tang", 3);
	aref_curve_btang = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "btang", 3);
	aref_curve_up = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "up", 3);
	aref_curve_twist = curve_gdp->findFloatTuple(GA_ATTRIB_POINT, "twist", 1);
	if (use_width)
	{
        aref_curve_width = curve_gdp->findFloatTuple(GA_ATTRIB_POINT, "width", 1);
        hndl_curve_width = aref_curve_width.getAttribute();
	}
	hndl_curve_tang = aref_curve_tang.getAttribute();
	hndl_curve_btang = aref_curve_btang.getAttribute();
	hndl_curve_up = aref_curve_up.getAttribute();
	hndl_curve_twist = aref_curve_twist.getAttribute();
	hndl_curve_p = curve_gdp->getP();
	GEO_Curve *geocurve_prim = static_cast<GEO_Curve*>(curve_geo_prim);
	computeCurveAttributes(geocurve_prim, time);

	float arclen = geocurve_prim->calcPerimeter();
	unsigned int curve_num_points = geocurve_prim->getPointRange().getEntries();

	// Main
    GA_RWPageHandleV3 hndl_geo_n;
	GA_RWPageHandleV3 hndl_geo_p = gdp->getP();
	GA_RWAttributeRef aref_geo_n = gdp->findNormalAttribute(GA_ATTRIB_POINT);
	if (aref_geo_n.isValid())
        hndl_geo_n = aref_geo_n.getAttribute();

	UT_BoundingBox bbox;
	gdp->getBBox(&bbox);
	// Find object z axis
	UT_Vector3 axis_pt0, axis_pt1;
	computeBboxAxis(bbox, axis_pt0, axis_pt1);
    float stretch_mult;
	float object_sizez = bbox.sizeZ();
    if (stretch_tolen)
        stretch_mult = arclen / object_sizez;
    else
        stretch_mult = (1.0 - PARM_STRETCH(time) * -1);
	object_sizez *= stretch_mult;
	float segment_length = arclen / curve_num_points;
	float step = object_sizez / segment_length; // how many cuve points in object length

    // Vector Attribs to reorient
    GA_AttributeRefMap aref_vecattribs((GA_Detail &)gdp);
    if (PARM_TRANSFORM_VECTORS())
    {
    	UT_String vecattribs_str;
    	PARM_REORIENT_ATTRIBS(vecattribs_str);
    	if (vecattribs_str.length() != 0)
    	{
    		UT_Array<GA_Attribute *> vattribs_array;
    		GOP_AttribListParse::parseAttribList(gdp->pointAttribs(), vecattribs_str, vattribs_array);
    		if (!vattribs_array.isEmpty())
    		{
    			for(exint i = 0; i < vattribs_array.entries(); ++i)
    			{
    				GA_Attribute *attr = vattribs_array(i);
    				attr->setTypeInfo(GA_TYPE_VECTOR);
                    aref_vecattribs.appendDest(attr);
    			}
    		}
    	}
    }

	// Deform
	UT_Vector3D nextCurveP, prevCurveP, lerpCurveP;
	UT_Vector3D nextCurveTang, prevCurveTang, lerpCurveTang;
	UT_Vector3D nextCurveBtang, prevCurveBtang, lerpCurveBtang;
	UT_Vector3D nextCurveUp, prevCurveUp, lerpCurveUp;
	UT_Vector3  projection_point, projection_direction;
    GA_Offset curve_offset;
    GA_Offset block_offset_start, block_offset_end;
	GA_IndexMap curveIndexMap = curve_gdp->getIndexMap(GA_ATTRIB_POINT);
	for(GA_Iterator it(gdp->getPointRange()); it.blockAdvance(block_offset_start, block_offset_end);)
	{
		hndl_geo_p.setPage(block_offset_start);
		hndl_geo_n.setPage(block_offset_start);
		for (GA_Offset ptof = block_offset_start; ptof < block_offset_end; ptof++)
		{
			// Find point projection on object axis
			UT_Vector3 axis_vector = axis_pt1 - axis_pt0;
			axis_vector.normalize();
			UT_Vector3 t0 = hndl_geo_p.get(ptof) - axis_pt0;
			float t0_len = t0.length();
			t0.normalize();
			float angle = t0.dot(axis_vector);
			float projection = angle * t0_len;
			projection_point = axis_pt0 + axis_vector * projection;
			projection_direction = hndl_geo_p.get(ptof) - projection_point;
        	float bbox_relpos = pointRelativeToBbox(bbox, hndl_geo_p.get(ptof), 2);
        	float u_position_on_curve = bbox_relpos * step;
        	u_position_on_curve += PARM_OFFSET(time) * (curve_num_points - 1);

        	unsigned int next_curve_pointnum = SYSmin(SYSceil(u_position_on_curve), (float) curve_num_points - 1);
        	unsigned int prev_curve_pointnum = SYSmin(SYSfloor(u_position_on_curve), (float) curve_num_points - 1);
        	float fraction = SYSfrac(u_position_on_curve);

        	// Import curve attribs
        	// Previous point
        	curve_offset = curveIndexMap.offsetFromIndex(next_curve_pointnum);
        	nextCurveP = hndl_curve_p.get(curve_offset);
        	nextCurveTang = hndl_curve_tang.get(curve_offset);
        	nextCurveBtang = hndl_curve_btang.get(curve_offset);
        	nextCurveUp = hndl_curve_up.get(curve_offset);

        	// Next point
        	curve_offset = curveIndexMap.offsetFromIndex(prev_curve_pointnum);
        	prevCurveP = hndl_curve_p.get(curve_offset);
        	prevCurveTang = hndl_curve_tang.get(curve_offset);
        	prevCurveBtang = hndl_curve_btang.get(curve_offset);
        	prevCurveUp = hndl_curve_up.get(curve_offset);

        	// Interpolated values
        	lerpCurveP = lerp(nextCurveP, prevCurveP, 1 - fraction);
        	lerpCurveTang = lerp(nextCurveTang, prevCurveTang, 1 - fraction);
        	lerpCurveBtang = lerp(nextCurveBtang, prevCurveBtang, 1 - fraction);
        	lerpCurveUp = lerp(nextCurveUp, prevCurveUp, 1 - fraction);

        	// Comstruct coordinate system
        	UT_Matrix3D curve_basis(lerpCurveBtang[0], lerpCurveBtang[1], lerpCurveBtang[2],
        			lerpCurveUp[0], lerpCurveUp[1], lerpCurveUp[2],
        			lerpCurveTang[0], lerpCurveTang[1], lerpCurveTang[2]);


        	projection_direction *= curve_basis;
        	if (use_width && hndl_curve_width.isValid())
        	{
        		double w1, w2;
        	    curve_offset = curveIndexMap.offsetFromIndex(next_curve_pointnum);
        	    w1 = hndl_curve_width.get(curve_offset);
        	    curve_offset = curveIndexMap.offsetFromIndex(prev_curve_pointnum);
        	    w2 = hndl_curve_width.get(curve_offset);
        		projection_direction *= lerp(w1, w2, (1 - fraction));
        	}
        	lerpCurveP += projection_direction;
        	hndl_geo_p.set(ptof, lerpCurveP);

        	if (PARM_TRANSFORM_VECTORS())
        	{
        		if (aref_vecattribs.entries() > 0)
        		{
        			UT_Matrix4D m,im;
        			m = curve_basis;
        			m.invert(im);
            	    aref_vecattribs.transform(m, im, GA_ATTRIB_POINT, ptof);
        		}
        	}
		}

	}
    if (recompute_n && hndl_geo_n.isValid())
    	gdp->normal();
	unlockInputs();
	return error();
}

void
newSopOperator(OP_OperatorTable *table)
{
	table->addOperator(
		new OP_Operator("path_deform",
						"PathDeform",
						PathDeform::MyConstructor,
						PathDeform::parmsTemplatesList,
						1,
						2,
						0,
						0,
						inputLabels));
}
