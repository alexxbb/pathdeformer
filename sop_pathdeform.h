/*
 * sop_pathdeform.h
 *
 *  Created on: Aug 4, 2014
 *      Author: alex
 */

#ifndef SOP_PATHDEFORM_H_
#define SOP_PATHDEFORM_H_

#include <UT/UT_DSOVersion.h>
#include <OP/OP_Node.h>
#include <SOP/SOP_Node.h>
#include <OP/OP_Parameters.h>

class PathDeform: public SOP_Node
{
public:
	PathDeform(OP_Network *net, const char *name, OP_Operator *op);
	virtual ~PathDeform();

	static OP_Node *MyConstructor(OP_Network *net, const char*, OP_Operator *op);
	static PRM_Template parmsTemplatesList[];
	static const char myInputLabels[2];

protected:
	OP_ERROR cookMySop(OP_Context &context);
	virtual bool updateParmsFlags();
	void computeBboxAxis(const int &axis, UT_Vector3 &pt0, UT_Vector3 &pt1);
	float pointRelativeToBbox(const UT_Vector3 &pt, const int &axis);

private:
	GA_RWHandleV3 hndl_curve_tang;
	GA_RWHandleV3 hndl_curve_btang;
	GA_RWHandleV3 hndl_curve_up;
	GA_ROHandleV3 hndl_curve_p;
	GA_ROHandleF hndl_curve_twist;
	GA_ROHandleF hndl_curve_width;
	void computeCurveAttributes(const GEO_Curve *curve_prim, fpreal time);
	int PARM_USEUPVECTOR() {return evalInt("use_up_vector", 0, 0);}
	int PARM_USETWIST() {return evalInt("use_curve_twist", 0, 0);}
	int PARM_USEWIDTH() {return evalInt("use_curve_width", 0, 0);}
	int PARM_STRETCH_TOLEN() {return evalInt("stretch_to_len", 0, 0);}
	int PARM_AXIS() {return evalInt(PRMaxisName.getToken(), 0, 0); }
	fpreal PARM_UPX(fpreal t) { return evalFloat("upvector", 0, t); }
	fpreal PARM_UPY(fpreal t) { return evalFloat("upvector", 1, t); }
	fpreal PARM_UPZ(fpreal t) { return evalFloat("upvector", 2, t); }
	float PARM_OFFSET(fpreal t) {return evalFloat("offset",  0, t); }
	float PARM_ROLL(fpreal t) {return evalFloat("roll",  0, t); }
	float PARM_STRETCH(fpreal t) {return evalFloat("stretch",  0, t); }
    void PARM_REORIENT_ATTRIBS(UT_String &str) {evalString(str, "vattribs", 0, 0);}
    int PARM_DEFORM_VECTORS() {return evalInt("deform_vattribs", 0, 0);}
    int PARM_ADD_BASIS_ATTR() {return evalInt("add_basis_attribs", 0, 0);}

    int PARM_COMPUTE_N() {return evalInt("recompute_n", 0, 0);}

    UT_Vector3 bbox_min, bbox_max;

};


class ThreadedDeform {
public:
	ThreadedDeform(GA_Attribute *attr_geo_p,
	GA_Attribute *attr_geo_n,
	GA_Attribute *attr_direction,
	GA_Attribute *attr_normal,
	GA_Attribute *attr_up,
	GA_RWHandleV3 &hndl_curve_tang,
	GA_RWHandleV3 &hndl_curve_btang,
	GA_RWHandleV3 &hndl_curve_up,
	const GA_ROHandleV3 &hndl_curve_p,
	const GA_ROHandleF &hndl_curve_twist,
	const GA_ROHandleF &hndl_curve_width,
	GA_AttributeRefMap &aref_map,
	const GA_IndexMap &curveIndexMap,
	const int &use_width,
	const int &stretch_tolen,
	const int &deform_vattribs,
	const int &axis,
	const float &offset,
	const float &step,
	const unsigned int &curve_num_points,
	const UT_BoundingBox &bbox,
	const UT_Vector3 &axis_vector,
	const UT_Vector3 &axis_pt0,
	const UT_Vector3 &axis_pt1):

	attr_geo_p(attr_geo_p),
		attr_geo_n(attr_geo_n),
		attr_direction(attr_direction),
		attr_normal(attr_normal),
		attr_up(attr_up),
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
		bbox(bbox),
		axis_vector(axis_vector),
		axis_pt0(axis_pt0),
		axis_pt1(axis_pt1)
	{

	}


	void operator()(const GA_SplittableRange &sr) const;

	private:
		GA_Attribute *attr_geo_p;
		GA_Attribute *attr_geo_n;
		GA_Attribute *attr_direction;
		GA_Attribute *attr_normal;
		GA_Attribute *attr_up;
		GA_RWHandleV3 hndl_curve_tang;
		GA_RWHandleV3 hndl_curve_btang;
		GA_RWHandleV3 hndl_curve_up;
		GA_ROHandleV3 hndl_curve_p;
		GA_ROHandleF hndl_curve_twist;
		GA_ROHandleF hndl_curve_width;
		GA_AttributeRefMap aref_map;
		const GA_IndexMap &curveIndexMap;
		UT_BoundingBox bbox;

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
#endif /* SOP_PATHDEFORM_H_ */
