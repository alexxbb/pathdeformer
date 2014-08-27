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

private:
	GA_RWAttributeRef aref_curve_tang;
	GA_RWAttributeRef aref_curve_btang;
	GA_RWAttributeRef aref_curve_up;
	GA_RWAttributeRef aref_curve_twist;
	GA_RWAttributeRef aref_curve_width;

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
	fpreal PARM_UPX(fpreal t) { return evalFloat("upvector", 0, t); }
	fpreal PARM_UPY(fpreal t) { return evalFloat("upvector", 1, t); }
	fpreal PARM_UPZ(fpreal t) { return evalFloat("upvector", 2, t); }
	float PARM_OFFSET(fpreal t) {return evalFloat("offset",  0, t); }
	float PARM_STRETCH(fpreal t) {return evalFloat("stretch",  0, t); }
    void PARM_REORIENT_ATTRIBS(UT_String &str) {evalString(str, "vattribs", 0, 0);}
    int PARM_TRANSFORM_VECTORS() {return evalInt("transform_vattribs", 0, 0);}
    int PARM_COMPUTE_N() {return evalInt("recompute_n", 0, 0);}

};

#endif /* SOP_PATHDEFORM_H_ */
