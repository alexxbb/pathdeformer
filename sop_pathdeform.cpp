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
#include <UT/UT_Vector3.h>
#include <UT/UT_ParallelUtil.h>
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
static PRM_Name deformVattribs("deform_vattribs", "Deform Vector Attributes");
static PRM_Name recompute_normals("recompute_n", "Recompute Point Normals");

static PRM_Range stretchRange(PRM_RANGE_RESTRICTED, -1, PRM_RANGE_UI, 2);

PRM_Template
PathDeform::parmsTemplatesList[] =
{
	PRM_Template(PRM_ORD, 1, &PRMaxisName, PRMtwoDefaults, &PRMaxisMenu),
	PRM_Template(PRM_TOGGLE_E, 1, &useUpVector, PRMzeroDefaults),
	PRM_Template(PRM_XYZ, 3, &PRMupVectorName, PRMyaxisDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &useCurveTwist, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &useCurveWidth, PRMoneDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &recompute_normals, PRMzeroDefaults),
	PRM_Template(PRM_TOGGLE_E, 1, &deformVattribs, PRMzeroDefaults),
    PRM_Template(PRM_STRING, 1, &vecAttribs, 0),
    PRM_Template(PRM_TOGGLE_E, 1, &stretch_tolen, PRMzeroDefaults),
    PRM_Template(PRM_FLT_J, 1, &stretch, PRMzeroDefaults, 0, &stretchRange),
	PRM_Template(PRM_FLT_J, 1, &PRMoffsetName, PRMzeroDefaults),
	PRM_Template(PRM_FLT_J, 1, &PRMrollName, PRMzeroDefaults),
	PRM_Template(),
};

bool
PathDeform::updateParmsFlags()
{
	bool changes = false;
//	changes |= enableParm(PRMupVectorName.getToken(), !PARM_USENORMALS());
	changes |= setVisibleState(PRMupVectorName.getToken(), PARM_USEUPVECTOR());
    changes |= enableParm(stretch.getToken(), !PARM_STRETCH_TOLEN());
    changes |= enableParm(vecAttribs.getToken(), PARM_DEFORM_VECTORS());
	return changes;
}


float
PathDeform::pointRelativeToBbox(const UT_Vector3 &pt, const int &axis)
{
	return SYSfit(pt[axis], bbox_min[axis], bbox_max[axis], 0, 1);
}



void
PathDeform::computeBboxAxis(const int &axis, UT_Vector3 &pt0, UT_Vector3 &pt1)
{
	float x,y;
	switch (axis)
	{
	case 0:
		x = bbox_min[2] + bbox_max[2];
		y = bbox_min[1] + bbox_max[1];
		pt0.assign(bbox_min[axis], x, y);
		pt1.assign(bbox_max[axis], x, y);
		break;
	case 1:
		x = bbox_min[0] + bbox_max[0];
		y = bbox_min[2] + bbox_max[2];
		pt0.assign(x, bbox_min[axis], y);
		pt1.assign(x, bbox_max[axis], y);
		break;
	case 2:
		x = bbox_min[0] + bbox_max[0];
		y = bbox_min[1] + bbox_max[1];
		pt0.assign(x, y, bbox_min[axis]);
		pt1.assign(x, y, bbox_max[axis]);
		break;
	}
}


void
PathDeform::computeCurveAttributes(const GEO_Curve *curve_prim, fpreal time)
{
	int use_curve_twist = PARM_USETWIST();
	float roll_parm = PARM_ROLL(time);
	UT_Vector3 prevP, curP, nextP, tang, btang, up, avg_normal;
	float roll_angle;

	if (PARM_USEUPVECTOR())
		avg_normal.assign(PARM_UPX(time), PARM_UPY(time), PARM_UPZ(time));
	else
		((GU_Curve *)curve_prim->castTo())->curveNormal(avg_normal);

	avg_normal.normalize();
	short int npts = curve_prim->getPointRange().getEntries();
	for(GA_Iterator it(curve_prim->getPointRange()); !it.atEnd(); it.advance())
	{
		roll_angle = 0.0;
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

		if (use_curve_twist || roll_parm > 0.0)
		{
			if (use_curve_twist && hndl_curve_twist.isValid())
				roll_angle = hndl_curve_twist.get(ptof);
			if (roll_parm > 0.0)
				roll_angle += roll_parm * 360.0;
            UT_QuaternionD quat(SYSdegToRad(roll_angle), tang);
			btang = quat.rotate(btang);
			up = quat.rotate(up);
		}

		hndl_curve_tang.set(ptof, tang);
		hndl_curve_btang.set(ptof, btang);
		hndl_curve_up.set(ptof, up);

	}
}

class ThreadedDeform
{
public:
	ThreadedDeform(GA_RWAttributeRef &aref_geo_p,
				   GA_RWAttributeRef &aref_geo_n,
				   GA_RWHandleV3 &hndl_curve_tang,
				   GA_RWHandleV3 &hndl_curve_btang,
				   GA_RWHandleV3 &hndl_curve_up,
				   GA_ROHandleV3 &hndl_curve_p,
				   GA_ROHandleF &hndl_curve_twist,
				   GA_ROHandleF &hndl_curve_width,
				   GA_AttributeRefMap &aref_map,
				   GA_IndexMap &curveIndexMap,
				   int &use_width,
				   int &stretch_tolen,
				   int &deform_vattribs,
				   int &axis,
				   float &offset,
				   float &step,
				   unsigned int &curve_num_points,
				   UT_Vector3 &bbox_min,
				   UT_Vector3 &bbox_max,
				   UT_Vector3 &axis_vector,
				   UT_Vector3 &axis_pt0,
				   UT_Vector3 &axis_pt1):
					   aref_geo_p(aref_geo_p),
					   aref_geo_n(aref_geo_n),
					   hndl_curve_tang(hndl_curve_tang),
					   hndl_curve_btang(hndl_curve_btang),
					   hndl_curve_up(hndl_curve_up),
					   hndl_curve_p(hndl_curve_p),
					   hndl_curve_twist(hndl_curve_twist),
					   hndl_curve_width(hndl_curve_width),
					   curveIndexMap(curveIndexMap),
					   aref_map(aref_map),
					   use_width(use_width),
					   stretch_tolen(stretch_tolen),
					   deform_vattribs(deform_vattribs),
					   axis(axis),
					   offset(offset),
					   step(step),
					   curve_num_points(curve_num_points),
					   bbox_min(bbox_min),
					   bbox_max(bbox_max),
					   axis_vector(axis_vector),
					   axis_pt0(axis_pt0),
					   axis_pt1(axis_pt1)
	{

	};
	~ThreadedDeform(){};

	float
	pointRelativeToBbox(const UT_Vector3 &pt, const int &axis) const
	{
		return SYSfit(pt[axis], bbox_min[axis], bbox_max[axis], 0, 1);
	}

	void operator()(const GA_SplittableRange &sr) const
	{

		UT_Vector3D nextCurveP, prevCurveP, lerpCurveP;
		UT_Vector3D nextCurveT, prevCurveT, lerpCurveT;
		UT_Vector3D nextCurveBT, prevCurveBT, lerpCurveBT;
		UT_Vector3D nextCurveUp, prevCurveUp, lerpCurveUp;
		UT_Vector3  projection_point, projection_direction;
		UT_Matrix3D curve_basis;
		GA_RWPageHandleV3 hndl_geo_n(aref_geo_n.getAttribute());
		GA_RWPageHandleV3 hndl_geo_p(aref_geo_p.getAttribute());

		for (GA_PageIterator pit = sr.beginPages(); !pit.atEnd(); ++pit)
		{
            GA_Offset block_offset_start, block_offset_end;
			for(GA_Iterator it(pit.begin()); it.blockAdvance(block_offset_start, block_offset_end);)
			{
				hndl_geo_p.setPage(block_offset_start);
				hndl_geo_n.setPage(block_offset_start);
				for (GA_Offset ptof = block_offset_start; ptof < block_offset_end; ++ptof)
				{
					// Find point projection on object axis
					UT_Vector3 t0 = hndl_geo_p.get(ptof) - axis_pt0;
					float t0_len = t0.length();
					t0.normalize();
					float angle = t0.dot(axis_vector);
					float projection = angle * t0_len;
					projection_point = axis_pt0 + axis_vector * projection;
					projection_direction = hndl_geo_p.get(ptof) - projection_point;
					float bbox_relpos = pointRelativeToBbox(hndl_geo_p.get(ptof), axis);
					float u_position_on_curve = bbox_relpos * step;
					u_position_on_curve += offset * (curve_num_points - 1);

					unsigned int next_curve_pointnum = SYSmin(SYSceil(u_position_on_curve), (float) curve_num_points - 1);
					unsigned int prev_curve_pointnum = SYSmin(SYSfloor(u_position_on_curve), (float) curve_num_points - 1);
					double fraction = SYSfrac(u_position_on_curve);

					// Import curve attribs
					// Previous point
					GA_Offset curve_offset = curveIndexMap.offsetFromIndex(next_curve_pointnum);
					nextCurveP = hndl_curve_p.get(curve_offset);
					nextCurveT = hndl_curve_tang.get(curve_offset);
					nextCurveBT = hndl_curve_btang.get(curve_offset);
					nextCurveUp = hndl_curve_up.get(curve_offset);

					// Next point
					curve_offset = curveIndexMap.offsetFromIndex(prev_curve_pointnum);
					prevCurveP = hndl_curve_p.get(curve_offset);
					prevCurveT = hndl_curve_tang.get(curve_offset);
					prevCurveBT = hndl_curve_btang.get(curve_offset);
					prevCurveUp = hndl_curve_up.get(curve_offset);

					// Interpolated values
					lerpCurveP = SYSlerp<double>(nextCurveP, prevCurveP, 1 - fraction);
					lerpCurveT = SYSlerp<double>(nextCurveT, prevCurveT, 1 - fraction);
					lerpCurveBT = SYSlerp<double>(nextCurveBT, prevCurveBT, 1 - fraction);
					lerpCurveUp = SYSlerp<double>(nextCurveUp, prevCurveUp, 1 - fraction);

					// Comstruct coordinate system
					switch (axis)
					{
						case 0:
							curve_basis = UT_Matrix3D(lerpCurveT[0], lerpCurveT[1], lerpCurveT[2],
											lerpCurveUp[0], lerpCurveUp[1], lerpCurveUp[2],
											-lerpCurveBT[0], -lerpCurveBT[1], -lerpCurveBT[2]);
							break;
						case 1:
							curve_basis = UT_Matrix3D(lerpCurveUp[0], lerpCurveUp[1], lerpCurveUp[2],
											lerpCurveT[0], lerpCurveT[1], lerpCurveT[2],
											lerpCurveBT[0], lerpCurveBT[1], lerpCurveBT[2]);
							break;
						case 2:
							curve_basis = UT_Matrix3D(lerpCurveBT[0], lerpCurveBT[1], lerpCurveBT[2],
											lerpCurveUp[0], lerpCurveUp[1], lerpCurveUp[2],
											-lerpCurveT[0], -lerpCurveT[1], -lerpCurveT[2]);
							break;
					}
					projection_direction *= curve_basis;
					if (use_width && hndl_curve_width.isValid())
					{
						double w1, w2;
						curve_offset = curveIndexMap.offsetFromIndex(next_curve_pointnum);
						w1 = hndl_curve_width.get(curve_offset);
						curve_offset = curveIndexMap.offsetFromIndex(prev_curve_pointnum);
						w2 = hndl_curve_width.get(curve_offset);
						projection_direction *= SYSlerp(w1, w2, (1 - fraction));
					}
					lerpCurveP += projection_direction;
					hndl_geo_p.set(ptof, lerpCurveP);

					if (deform_vattribs)
					{
						if (aref_map.entries() > 0)
						{
							UT_Matrix4D m,im;
							m = curve_basis;
							m.invert(im);
							aref_map.transform(m, im, GA_ATTRIB_POINT, ptof);
						}
					}
				}

		}

	}
};
private:
	GA_RWAttributeRef aref_geo_p;
	GA_RWAttributeRef aref_geo_n;
	GA_RWHandleV3 hndl_curve_tang;
	GA_RWHandleV3 hndl_curve_btang;
	GA_RWHandleV3 hndl_curve_up;
	GA_ROHandleV3 hndl_curve_p;
	GA_ROHandleF hndl_curve_twist;
	GA_ROHandleF hndl_curve_width;
	GA_AttributeRefMap aref_map;
	GA_IndexMap curveIndexMap;
	UT_Vector3 bbox_min, bbox_max;

	int use_width;
	int stretch_tolen;
	int deform_vattribs;
	int axis;
	float offset;
	float step;
	unsigned int curve_num_points;

	UT_Vector3 axis_vector;
	UT_Vector3 axis_pt0;
	UT_Vector3 axis_pt1;

};



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

	duplicatePointSource(0, context, gdp);
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
    int deform_vattribs = PARM_DEFORM_VECTORS();
    int axis = PARM_AXIS();
    float offset = PARM_OFFSET(time);
	// Geometry attributes
	GA_RWAttributeRef aref_curve_tang = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "tang", 3);
	GA_RWAttributeRef aref_curve_btang = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "btang", 3);
	GA_RWAttributeRef aref_curve_up = curve_gdp->addFloatTuple(GA_ATTRIB_POINT, "up", 3);
	GA_RWAttributeRef aref_geo_n = gdp->findNormalAttribute(GA_ATTRIB_POINT);
	GA_RWAttributeRef aref_geo_p = gdp->findFloatTuple(GA_ATTRIB_POINT,  "P", 3);

	hndl_curve_tang = aref_curve_tang.getAttribute();
	hndl_curve_btang = aref_curve_btang.getAttribute();
	hndl_curve_up = aref_curve_up.getAttribute();
	hndl_curve_p = curve_gdp->getP();
	GA_RWPageHandleV3 hndl_geo_p(gdp->getP());
	GA_RWPageHandleV3 hndl_geo_n;
	if (aref_geo_n.isValid())
		GA_RWPageHandleV3 hndl_geo_n(aref_geo_n.getAttribute());
	hndl_curve_twist = curve_gdp->findPointAttribute("twist").getAttribute();
	hndl_curve_width = curve_gdp->findPointAttribute("width").getAttribute();

	GEO_Curve *geocurve_prim = static_cast<GEO_Curve*>(curve_geo_prim);
	computeCurveAttributes(geocurve_prim, time);
	float arclen = geocurve_prim->calcPerimeter();
	unsigned int curve_num_points = geocurve_prim->getPointRange().getEntries();

	// Main
	UT_BoundingBox bbox;
	UT_Vector3 axis_pt0, axis_pt1;
	gdp->getBBox(&bbox);
	bbox_min = bbox.minvec();
	bbox_max = bbox.maxvec();
	computeBboxAxis(axis, axis_pt0, axis_pt1);
	UT_Vector3 axis_vector = axis_pt1 - axis_pt0;
	axis_vector.normalize();

    float stretch_mult;
	float object_axis_size = bbox.sizeAxis(axis);
    if (stretch_tolen)
        stretch_mult = arclen / object_axis_size;
    else
        stretch_mult = (1.0 - PARM_STRETCH(time) * -1);
	object_axis_size *= stretch_mult;
	float segment_length = arclen / curve_num_points;
	float step = object_axis_size / segment_length; // how many cuve points in object length

    // Parse string parameter and make AttribRefMap for vector attribs to deform
    GA_AttributeRefMap aref_map((GA_Detail &)gdp);
    GA_AttributeRefMap::ThreadHarden harden(aref_map); // make aref_map thread safe
    if (PARM_DEFORM_VECTORS())
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
                    aref_map.appendDest(attr);
    			}
    		}
    	}
    }

	// Deformation.
    const GA_SplittableRange sr(gdp->getPointRange());
	GA_IndexMap curveIndexMap = curve_gdp->getIndexMap(GA_ATTRIB_POINT);
	ThreadedDeform td(
			aref_geo_p,
			aref_geo_n,
			hndl_curve_tang,
			hndl_curve_btang,
			hndl_curve_up,
			hndl_curve_p,
			hndl_curve_twist,
			hndl_curve_width,
			aref_map,
			curveIndexMap,
			use_width,
			stretch_tolen,
			deform_vattribs,
			axis,
			offset,
			step,
			curve_num_points,
			bbox_min,
			bbox_max,
			axis_vector,
			axis_pt0,
			axis_pt1);

	UTparallelFor(sr, td);
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
