/*
 * Copyright (c) 2004, Ben Supnik and Chris Serio.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "XPMPPlaneRenderer.h"
#include "XPMPMultiplayer.h"
#include "XPMPMultiplayerCSL.h"
#include "XPMPMultiplayerCSLOffset.h"
#include "XPMPMultiplayerVars.h"
#include "XPMPMultiplayerObj.h"
#include "XPMPMultiplayerObj8.h"

#include "XPLMGraphics.h"
#include "XPLMDisplay.h"
#include "XPLMCamera.h"
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"

#include <stdio.h>
#include <math.h>
#include <algorithm>

#if IBM
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <cstring>
#include <vector>
#include <string>
#include <set>
#include <map>

// Turn this on to get a lot of diagnostic info on who's visible, etc.
#define 	DEBUG_RENDERER 0
// Turn this on to put rendering stats in datarefs for realtime observatoin.
#define		RENDERER_STATS 0

// Maximum altitude difference in feet for TCAS blips
#define		MAX_TCAS_ALTDIFF		10000

// Even in good weather we don't want labels on things
// that we can barely see.  Cut labels at 5 km.
#define		MAX_LABEL_DIST			5000.0

extern bool	gHasControlOfAIAircraft;
constexpr float FAR_AWAY_VAL_GL = 9999999.9f;    // don't dare using NAN...but with this coordinate for x/y/z a plane should be far out and virtually invisible
float fNull[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
struct multiDataRefsTy {
    XPLMDataRef X;          // position
    XPLMDataRef Y;
    XPLMDataRef Z;
    
    XPLMDataRef v_x;        // cartesian velocities
    XPLMDataRef v_y;
    XPLMDataRef v_z;
    
    XPLMDataRef pitch;      // theta
    XPLMDataRef roll;       // phi
    XPLMDataRef heading;    // psi
    
    XPLMDataRef gear;       // gear_deploy[10]
    XPLMDataRef flap;       // flap_ratio
    XPLMDataRef flap2;      // flap_ratio2
    XPLMDataRef spoiler;    // spoiler_ratio
    XPLMDataRef speedbrake; // speedbrake
    XPLMDataRef slat;       // slat_ratio
    XPLMDataRef wingSweep;  // wing_sweep
    XPLMDataRef throttle;   // throttle[8]
    XPLMDataRef yoke_pitch; // yolk_pitch
    XPLMDataRef yoke_roll;  // yolk_roll
    XPLMDataRef yoke_yaw;   // yolk_yaw
    
    XPLMDataRef bcnLights;  // beacon_lights_on
    XPLMDataRef landLights; // landing_lights_on
    XPLMDataRef navLights;  // nav_lights_on
    XPLMDataRef strbLights; // strobe_lights_on
    XPLMDataRef taxiLights; // taxi_light_on
    
    // Shared data for providing textual info (see XPMPInfoTexts_t)
    XPLMDataRef infoTailNum;        // tailnum
    XPLMDataRef infoIcaoAcType;     // ICAO
    XPLMDataRef infoManufacturer;   // manufacturer
    XPLMDataRef infoModel;          // model
    XPLMDataRef infoIcaoAirline;    // ICAOairline
    XPLMDataRef infoAirline;        // airline
    XPLMDataRef infoFlightNum;      // flightnum
    XPLMDataRef infoAptFrom;        // apt_from
    XPLMDataRef infoAptTo;          // apt_to
    
    bool        bSlotTaken = false; // during drawing: is this multiplayer plane idx used or not?
    
    // all OK?
    inline operator bool () const { return X && Y && Z && pitch && roll && heading; }
};

std::vector<multiDataRefsTy>        gMultiRef;

bool gDrawLabels = true;

struct cull_info_t {					// This struct has everything we need to cull fast!
	float	model_view[16];				// The model view matrix, to get from local OpenGL to eye coordinates.
	float	proj[16];					// Proj matrix - this is just a hack to use for gluProject.
	float	nea_clip[4];				// Four clip planes in the form of Ax + By + Cz + D = 0 (ABCD are in the array.)
	float	far_clip[4];				// They are oriented so the positive side of the clip plane is INSIDE the view volume.
	float	lft_clip[4];
	float	rgt_clip[4];
	float	bot_clip[4];
	float	top_clip[4];
};

static bool				gCullInfoInitialised = false;
static XPLMDataRef		projectionMatrixRef = nullptr;
static XPLMDataRef		modelviewMatrixRef = nullptr;
static XPLMDataRef		viewportRef = nullptr;

bool					gMSAAHackInitialised = false;
static XPLMDataRef  	gMSAAXRatioRef = nullptr;
static XPLMDataRef		gMSAAYRatioRef = nullptr;
static XPLMDataRef      gHDROnRef = nullptr;

static void
init_cullinfo()
{
	modelviewMatrixRef = XPLMFindDataRef("sim/graphics/view/modelview_matrix");
	projectionMatrixRef = XPLMFindDataRef("sim/graphics/view/projection_matrix");
	viewportRef = XPLMFindDataRef("sim/graphics/view/viewport");
	gCullInfoInitialised = true;
}


static void setup_cull_info(cull_info_t * i)
{
	if (!gCullInfoInitialised) {
		init_cullinfo();
	}
	// First, just read out the current OpenGL matrices...do this once at setup
	// because it's not the fastest thing to do.
	//
	// if our X-Plane version supports it, pull it from the daatrefs to avoid a
	// potential driver stall.
	if (!modelviewMatrixRef || !projectionMatrixRef) {
		glGetFloatv(GL_MODELVIEW_MATRIX, i->model_view);
		glGetFloatv(GL_PROJECTION_MATRIX, i->proj);
	} else {
		XPLMGetDatavf(modelviewMatrixRef, i->model_view, 0, 16);
		XPLMGetDatavf(projectionMatrixRef, i->proj, 0, 16);
	}

	// Now...what the heck is this?  Here's the deal: the clip planes have values in "clip" coordinates of: Left = (1,0,0,1)
	// Right = (-1,0,0,1), Bottom = (0,1,0,1), etc.  (Clip coordinates are coordinates from -1 to 1 in XYZ that the driver
	// uses.  The projection matrix converts from eye to clip coordinates.)
	//
	// How do we convert a plane backward from clip to eye coordinates?  Well, we need the transpose of the inverse of the
	// inverse of the projection matrix.  (Transpose of the inverse is needed to transform a plane, and the inverse of the
	// projection is the matrix that goes clip -> eye.)  Well, that cancels out to the transpose of the projection matrix,
	// which is nice because it means we don't need a matrix inversion in this bit of sample code.
	
	// So this nightmare down here is simply:
	// clip plane * transpose (proj_matrix)
	// worked out for all six clip planes.  If you squint you can see the patterns:
	// L:  1  0 0 1
	// R: -1  0 0 1
	// B:  0  1 0 1
	// T:  0 -1 0 1
	// etc.
	
	i->lft_clip[0] = i->proj[0]+i->proj[3];	i->lft_clip[1] = i->proj[4]+i->proj[7];	i->lft_clip[2] = i->proj[8]+i->proj[11];	i->lft_clip[3] = i->proj[12]+i->proj[15];
	i->rgt_clip[0] =-i->proj[0]+i->proj[3];	i->rgt_clip[1] =-i->proj[4]+i->proj[7];	i->rgt_clip[2] =-i->proj[8]+i->proj[11];	i->rgt_clip[3] =-i->proj[12]+i->proj[15];
	
	i->bot_clip[0] = i->proj[1]+i->proj[3];	i->bot_clip[1] = i->proj[5]+i->proj[7];	i->bot_clip[2] = i->proj[9]+i->proj[11];	i->bot_clip[3] = i->proj[13]+i->proj[15];
	i->top_clip[0] =-i->proj[1]+i->proj[3];	i->top_clip[1] =-i->proj[5]+i->proj[7];	i->top_clip[2] =-i->proj[9]+i->proj[11];	i->top_clip[3] =-i->proj[13]+i->proj[15];

	i->nea_clip[0] = i->proj[2]+i->proj[3];	i->nea_clip[1] = i->proj[6]+i->proj[7];	i->nea_clip[2] = i->proj[10]+i->proj[11];	i->nea_clip[3] = i->proj[14]+i->proj[15];
	i->far_clip[0] =-i->proj[2]+i->proj[3];	i->far_clip[1] =-i->proj[6]+i->proj[7];	i->far_clip[2] =-i->proj[10]+i->proj[11];	i->far_clip[3] =-i->proj[14]+i->proj[15];
}

static int sphere_is_visible(const cull_info_t * i, float x, float y, float z, float r)
{
	// First: we transform our coordinate into eye coordinates from model-view.
	float xp = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + i->model_view[12];
	float yp = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + i->model_view[13];
	float zp = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + i->model_view[14];

	// Now - we apply the "plane equation" of each clip plane to see how far from the clip plane our point is.
	// The clip planes are directed: positive number distances mean we are INSIDE our viewing area by some distance;
	// negative means outside.  So ... if we are outside by less than -r, the ENTIRE sphere is out of bounds.
	// We are not visible!  We do the near clip plane, then sides, then far, in an attempt to try the planes
	// that will eliminate the most geometry first...half the world is behind the near clip plane, but not much is
	// behind the far clip plane on sunny day.
	if ((xp * i->nea_clip[0] + yp * i->nea_clip[1] + zp * i->nea_clip[2] + i->nea_clip[3] + r) < 0)	return false;
	if ((xp * i->bot_clip[0] + yp * i->bot_clip[1] + zp * i->bot_clip[2] + i->bot_clip[3] + r) < 0)	return false;
	if ((xp * i->top_clip[0] + yp * i->top_clip[1] + zp * i->top_clip[2] + i->top_clip[3] + r) < 0)	return false;
	if ((xp * i->lft_clip[0] + yp * i->lft_clip[1] + zp * i->lft_clip[2] + i->lft_clip[3] + r) < 0)	return false;
	if ((xp * i->rgt_clip[0] + yp * i->rgt_clip[1] + zp * i->rgt_clip[2] + i->rgt_clip[3] + r) < 0)	return false;
	if ((xp * i->far_clip[0] + yp * i->far_clip[1] + zp * i->far_clip[2] + i->far_clip[3] + r) < 0)	return false;
	return true;
}

static float sphere_distance_sqr(const cull_info_t * i, float x, float y, float z)
{
	float xp = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + i->model_view[12];
	float yp = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + i->model_view[13];
	float zp = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + i->model_view[14];
	return xp*xp+yp*yp+zp*zp;
}

static void convert_to_2d(const cull_info_t * i, const int * vp, float x, float y, float z, float w, float * out_x, float * out_y)
{
	float xe = x * i->model_view[0] + y * i->model_view[4] + z * i->model_view[ 8] + w * i->model_view[12];
	float ye = x * i->model_view[1] + y * i->model_view[5] + z * i->model_view[ 9] + w * i->model_view[13];
	float ze = x * i->model_view[2] + y * i->model_view[6] + z * i->model_view[10] + w * i->model_view[14];
	float we = x * i->model_view[3] + y * i->model_view[7] + z * i->model_view[11] + w * i->model_view[15];

	float xc = xe * i->proj[0] + ye * i->proj[4] + ze * i->proj[ 8] + we * i->proj[12];
	float yc = xe * i->proj[1] + ye * i->proj[5] + ze * i->proj[ 9] + we * i->proj[13];
	//	float zc = xe * i->proj[2] + ye * i->proj[6] + ze * i->proj[10] + we * i->proj[14];
	float wc = xe * i->proj[3] + ye * i->proj[7] + ze * i->proj[11] + we * i->proj[15];
	
	xc /= wc;
	yc /= wc;
	//	zc /= wc;

	*out_x = static_cast<float>(vp[0]) + (1.0f + xc) * static_cast<float>(vp[2]) / 2.0f;
	*out_y = static_cast<float>(vp[1]) + (1.0f + yc) * static_cast<float>(vp[3]) / 2.0f;
}


#if RENDERER_STATS

static int		GetRendererStat(void * inRefcon)
{
	return *((int *) inRefcon);
}

#endif

static	int		gTotPlanes = 0;			// Counters
static	int		gACFPlanes = 0;			// Number of Austin's planes we drew in full
static	int		gNavPlanes = 0;			// Number of Austin's planes we drew with lights only
static	int		gOBJPlanes = 0;			// Number of our OBJ planes we drew in full

static	XPLMDataRef		gVisDataRef = NULL;		// Current air visiblity for culling.
static	XPLMDataRef		gAltitudeRef = NULL;	// Current aircraft altitude (for TCAS)

static XPLMProbeRef terrainProbe = NULL; // Probe to probe where the ground is for clamping


void			XPMPInitDefaultPlaneRenderer(void)
{
	terrainProbe = XPLMCreateProbe(xplm_ProbeY);
	
	// SETUP - mostly just fetch datarefs.

	if (!gCullInfoInitialised) {
		init_cullinfo();
	}
	gVisDataRef = XPLMFindDataRef("sim/graphics/view/visibility_effective_m");
	if (gVisDataRef == NULL) gVisDataRef = XPLMFindDataRef("sim/weather/visibility_effective_m");
	if (gVisDataRef == NULL)
		XPLMDebugString("WARNING: Default renderer could not find effective visibility in the sim.\n");

	if(gAltitudeRef == NULL) gAltitudeRef = XPLMFindDataRef("sim/flightmodel/position/elevation");

#if RENDERER_STATS
	XPLMRegisterDataAccessor("hack/renderer/planes", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gTotPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/navlites", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gNavPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/objects", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gOBJPlanes, NULL);
	XPLMRegisterDataAccessor("hack/renderer/acfs", xplmType_Int, 0, GetRendererStat, NULL,
							 NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
							 &gACFPlanes, NULL);
#endif		

	// We don't know how many multiplayer planes there are - fetch as many as we can.
    gMultiRef.clear();                  // just a safety measure against multi init
	char		buf[100];
	multiDataRefsTy	d;
    
#define FIND_PLANE_DR(membVar, dataRefTxt, PlaneNr)                            \
        sprintf(buf,"sim/multiplayer/position/plane%d_" dataRefTxt,PlaneNr);   \
        d.membVar = XPLMFindDataRef(buf);
#define SHARE_PLANE_DR(membVar, dataRefTxt, PlaneNr)                           \
        sprintf(buf,"sim/multiplayer/position/plane%d_" dataRefTxt,PlaneNr);   \
        if (XPLMShareData(buf, xplmType_Data, NULL, NULL))                     \
             d.membVar = XPLMFindDataRef(buf);                                 \
        else d.membVar = NULL;

    for (int n=1; true; n++)
	{
        // position
        FIND_PLANE_DR(X,            "x", n);
        FIND_PLANE_DR(Y,            "y", n);
        FIND_PLANE_DR(Z,            "z", n);
        // cartesian velocities
        FIND_PLANE_DR(v_x,          "v_x", n);
        FIND_PLANE_DR(v_y,          "v_y", n);
        FIND_PLANE_DR(v_z,          "v_z", n);
        // attitude
        FIND_PLANE_DR(pitch,        "the", n);
        FIND_PLANE_DR(roll,         "phi", n);
        FIND_PLANE_DR(heading,      "psi", n);
        // configuration
        FIND_PLANE_DR(gear,         "gear_deploy", n);
        FIND_PLANE_DR(flap,         "flap_ratio", n);
        FIND_PLANE_DR(flap2,        "flap_ratio2", n);
        FIND_PLANE_DR(spoiler,      "spoiler_ratio", n);
        FIND_PLANE_DR(speedbrake,   "speedbrake_ratio", n);
        FIND_PLANE_DR(slat,         "slat_ratio", n);
        if (!d.slat) {
            FIND_PLANE_DR(slat,     "sla1_ratio", n);
        }
        FIND_PLANE_DR(wingSweep,    "wing_sweep", n);
        FIND_PLANE_DR(throttle,     "throttle", n);
        FIND_PLANE_DR(yoke_pitch,   "yolk_pitch", n);
        FIND_PLANE_DR(yoke_roll,    "yolk_roll", n);
        FIND_PLANE_DR(yoke_yaw,     "yolk_yaw", n);
        // lights
        FIND_PLANE_DR(bcnLights,    "beacon_lights_on", n);
        FIND_PLANE_DR(landLights,   "landing_lights_on", n);
        FIND_PLANE_DR(navLights,    "nav_lights_on", n);
        FIND_PLANE_DR(strbLights,   "strobe_lights_on", n);
        FIND_PLANE_DR(taxiLights,   "taxi_light_on", n);
        
        // Shared data for providing textual info (see XPMPInfoTexts_t)
        SHARE_PLANE_DR(infoTailNum,         "tailnum", n);
        SHARE_PLANE_DR(infoIcaoAcType,      "ICAO", n);
        SHARE_PLANE_DR(infoManufacturer,    "manufacturer", n);
        SHARE_PLANE_DR(infoModel,           "model", n);
        SHARE_PLANE_DR(infoIcaoAirline,     "ICAOairline", n);
        SHARE_PLANE_DR(infoAirline,         "airline", n);
        SHARE_PLANE_DR(infoFlightNum,       "flightnum", n);
        SHARE_PLANE_DR(infoAptFrom,         "apt_from", n);
        SHARE_PLANE_DR(infoAptTo,           "apt_to", n);

        if (!d) break;
		gMultiRef.push_back(d);
	}
}

void XPMPDeinitDefaultPlaneRenderer() {
	XPLMDestroyProbe(terrainProbe);
    
    // Unshare shared data
#define UNSHARE_PLANE_DR(membVar, dataRefTxt, PlaneNr)                         \
    sprintf(buf,"sim/multiplayer/position/plane%d_" dataRefTxt,PlaneNr);       \
    XPLMUnshareData(buf, xplmType_Data, NULL, NULL);                           \
    d.membVar = NULL;
    
    char        buf[100];
    for (int n = 1; n <= gMultiRef.size(); n++)
    {
        multiDataRefsTy& d = gMultiRef[n-1];
        UNSHARE_PLANE_DR(infoTailNum,         "tailnum", n);
        UNSHARE_PLANE_DR(infoIcaoAcType,      "ICAO", n);
        UNSHARE_PLANE_DR(infoManufacturer,    "manufacturer", n);
        UNSHARE_PLANE_DR(infoModel,           "model", n);
        UNSHARE_PLANE_DR(infoIcaoAirline,     "ICAOairline", n);
        UNSHARE_PLANE_DR(infoAirline,         "airline", n);
        UNSHARE_PLANE_DR(infoFlightNum,       "flightnum", n);
        UNSHARE_PLANE_DR(infoAptFrom,         "apt_from", n);
        UNSHARE_PLANE_DR(infoAptTo,           "apt_to", n);
    }
}

void XPMPClearMultiplayerDataRefs (multiDataRefsTy& mdr)
{
    mdr.bSlotTaken = false;
    XPLMSetDataf(mdr.X, FAR_AWAY_VAL_GL);
    XPLMSetDataf(mdr.Y, FAR_AWAY_VAL_GL);
    XPLMSetDataf(mdr.Z, FAR_AWAY_VAL_GL);
    
    XPLMSetDataf(mdr.v_x, 0.0f);
    XPLMSetDataf(mdr.v_y, 0.0f);
    XPLMSetDataf(mdr.v_z, 0.0f);
    
    XPLMSetDataf(mdr.pitch, 0.0f);
    XPLMSetDataf(mdr.roll, 0.0f);
    XPLMSetDataf(mdr.heading, 0.0f);
    
    XPLMSetDatavf(mdr.gear, fNull, 0, 10);
    XPLMSetDataf(mdr.flap, 0.0f);
    XPLMSetDataf(mdr.flap2, 0.0f);
    XPLMSetDataf(mdr.spoiler, 0.0f);
    XPLMSetDataf(mdr.speedbrake, 0.0f);
    XPLMSetDataf(mdr.slat, 0.0f);
    XPLMSetDataf(mdr.wingSweep, 0.0f);
    XPLMSetDatavf(mdr.throttle, fNull, 0, 8);
    XPLMSetDataf(mdr.yoke_pitch, 0.0f);
    XPLMSetDataf(mdr.yoke_roll, 0.0f);
    XPLMSetDataf(mdr.yoke_yaw, 0.0f);
    
    XPLMSetDatai(mdr.bcnLights, 0);
    XPLMSetDatai(mdr.landLights, 0);
    XPLMSetDatai(mdr.navLights, 0);
    XPLMSetDatai(mdr.strbLights, 0);
    XPLMSetDatai(mdr.taxiLights, 0);
    
    // Shared data for providing textual info (see XPMPInfoTexts_t)
    char allNulls[100];
    memset (allNulls, 0, sizeof(allNulls));
    XPLMSetDatab(mdr.infoTailNum,       allNulls, 0, sizeof(XPMPInfoTexts_t::tailNum));
    XPLMSetDatab(mdr.infoIcaoAcType,    allNulls, 0, sizeof(XPMPInfoTexts_t::icaoAcType));
    XPLMSetDatab(mdr.infoManufacturer,  allNulls, 0, sizeof(XPMPInfoTexts_t::manufacturer));
    XPLMSetDatab(mdr.infoModel,         allNulls, 0, sizeof(XPMPInfoTexts_t::model));
    XPLMSetDatab(mdr.infoIcaoAirline,   allNulls, 0, sizeof(XPMPInfoTexts_t::icaoAirline));
    XPLMSetDatab(mdr.infoAirline,       allNulls, 0, sizeof(XPMPInfoTexts_t::airline));
    XPLMSetDatab(mdr.infoFlightNum,     allNulls, 0, sizeof(XPMPInfoTexts_t::flightNum));
    XPLMSetDatab(mdr.infoAptFrom,       allNulls, 0, sizeof(XPMPInfoTexts_t::aptFrom));
    XPLMSetDatab(mdr.infoAptTo,         allNulls, 0, sizeof(XPMPInfoTexts_t::aptTo));
}

// reset all (controlled) multiplayer dataRef values
void XPMPInitMultiplayerDataRefs() {
    if (gHasControlOfAIAircraft)
        for (multiDataRefsTy& mdr : gMultiRef)
            XPMPClearMultiplayerDataRefs(mdr);
}

/* correctYValue returns the clamped Z value given the input X, Y and Z and the
 * known vertical offset of the aircraft model.
*/
double
correctYValue(double inX, double inY, double inZ, double inModelYOffset)
{
	XPLMProbeInfo_t info;
	info.structSize = sizeof(XPLMProbeInfo_t);
	XPLMProbeResult res = XPLMProbeTerrainXYZ(terrainProbe,
		static_cast<float>(inX),
		static_cast<float>(inY),
		static_cast<float>(inZ),
		&info);
	if (res != xplm_ProbeHitTerrain) {
		return inY;
	}
	double minY = info.locationY + inModelYOffset;
	return (inY < minY) ? minY : inY;
}

// PlaneToRender struct: we prioritize planes radially by distance, so...
// we use this struct to remember one visible plane.  Once we've
// found all visible planes, we draw the closest ones.

struct	PlaneToRender_t {
	float					x;			// Positional info
	float					y;
	float					z;
	XPMPPlanePtr			plane;
	bool					full;		// Do we need to draw the full plane or just lites?
	bool					cull;		// Are we visible on screen?
	bool					tcas;		// Are we visible on TCAS?
	XPLMPlaneDrawState_t	state;		// Flaps, gear, etc.
	float					dist;
};
typedef	std::map<float, PlaneToRender_t>	RenderMap;


// *********************************
// Handling of AI/multiplayer Planes for TCAS, maps, camera plugins...
// *********************************
//
// We want a plane to keep its index as long as possible. This eases
// following it from other plugins.
// The plane's multiplayer idx is in XPMPPlane_t::multiIdx.
//
// There are usually 19 slots for multiplayer planes.
// But the user might have set up less actual AI planes.
// Some plugins (including XP's own map)consider this number,
// others doen't.
// Our approach: We make sure the first `modelCount` slots (lower part)
// are used by the closest a/c. The others (upper part) fill up with those
// father away.
// That certainly means that a/c still might switch AI slots
// if they move from lower to upper part or vice versa.
// To avoid too fast/too often switching of slots we allow change of slots
// only every 30s.

int GetAndReserveNextAISlotNr (XPMPPlane_t& plane)
{
    // safeguard: already has a slot?
    if (plane.multiIdx >= 0)
        return plane.multiIdx;
    
    // search for a free slot
    for (int idx = 0; idx < gMultiRef.size(); idx++)
        if (!gMultiRef[idx].bSlotTaken) {
            gMultiRef[idx].bSlotTaken = true;
            plane.multiIdx = idx;
            break;
        }

#ifdef SLOT_DEBUG
    if (plane.multiIdx >= 0) {
        char buf[200];
        snprintf(buf, sizeof(buf), XPMP_CLIENT_NAME ": ASSIGNING AI Slot %d for %s (%s of %s)\n",
                 plane.multiIdx+1, plane.livery.c_str(),
                 plane.icao.c_str(), plane.airline.c_str());
        XPLMDebugString(buf);
    }
#endif

    return plane.multiIdx;
}

// this is only a function for the debug output
inline void ClearAISlot (XPMPPlane_t& plane)
{
#ifdef SLOT_DEBUG
    if (plane.multiIdx >= 0) {
        char buf[200];
        snprintf(buf, sizeof(buf), XPMP_CLIENT_NAME ": RELEASING AI Slot %d of %s (%s of %s)\n",
                 plane.multiIdx+1, plane.livery.c_str(),
                 plane.icao.c_str(), plane.airline.c_str());
        XPLMDebugString(buf);
    }
#endif
    plane.multiIdx = -1;
}

void XPMPMultiplayerHandling (RenderMap& myPlanes)
{
    // If we don't control AI aircraft we bail out
    if (!gHasControlOfAIAircraft)
        return;
    
    // `modelCount` is the number of configured AI aircraft
    int modelCount, active, plugin;
    XPLMCountAircraft(&modelCount, &active, &plugin);
    modelCount--;               // first one's the user plane, here we don't count that

    // reset our bookkeeping on used multiplay idx
    for (multiDataRefsTy& iter: gMultiRef)
        iter.bSlotTaken = false;
    
    // Time of last slot switching activity
    static std::chrono::steady_clock::time_point tLastSlotSwitching;
    const std::chrono::steady_clock::time_point tNow = std::chrono::steady_clock::now();
    // only every few seconds rearrange slots, ie. add/remove planes or
    // move planes between lower and upper section of AI slots:
    if (tNow - tLastSlotSwitching > std::chrono::seconds(kSlotChangePeriod))
    {
        tLastSlotSwitching = tNow;
    
        // run over all known planes in order of prio/distance and
        // set those multiplayer idx used, which are taken
        int numTcasPlanes = 0;      // counts TCAS-relevant planes
        for (const RenderMap::value_type& pair: myPlanes)
        {
            // the multiplayer index stored with the plane:
            const int refIdx = pair.second.plane->multiIdx;

            // TCAS relevant and still AI slots available?
            if (pair.second.tcas && numTcasPlanes < gMultiRef.size())
            {
                // The lower part (close planes), shall display in the safe lower part of AI slots
                if (numTcasPlanes < modelCount)
                {
                    // current plane MUST get a slot in this range
                    if (0 <= refIdx && refIdx < modelCount && !gMultiRef[refIdx].bSlotTaken)
                        // it already has a reserved one, good
                        gMultiRef[refIdx].bSlotTaken = true;
                    else
                        // it will get one later (we still might need to free up lower part slots
                        ClearAISlot(*pair.second.plane);
                } else {
                    // Planes father away (upper part):
                    // If the plane has a reserved slot in the upper part it keeps it
                    if (modelCount <= refIdx && refIdx < gMultiRef.size() && !gMultiRef[refIdx].bSlotTaken)
                        gMultiRef[refIdx].bSlotTaken = true;
                    else
                        // Otherwise free it (this might free up lower part slots)
                        ClearAISlot(*pair.second.plane);    // it will get a slot later
                }

                // one more TCAS plane found
                numTcasPlanes++;
            }
            // not TCAS relevant or ran out of AI slots
            else
                ClearAISlot(*pair.second.plane);
        }           // loop all planes
    }               // if time for AI slot switching
    else
    {
        // only quickly do bookkeeping on used slots
        // (even without re-slotting above, planes can have just vanished)
        for (const RenderMap::value_type& pair: myPlanes) {
            const int idx = pair.second.plane->multiIdx;
            if (0 <= idx && idx < gMultiRef.size())
                gMultiRef[idx].bSlotTaken = true;
        }
    }
        
    // Fill multiplayer dataRefs for the planes relevant for TCAS
    int multiCount = 0;         // number of dataRefs set, just to exit loop early
    int maxMultiIdxUsed = 0;    // maximum AI index used
    
    for (RenderMap::value_type& pair: myPlanes) {
        XPMPPlane_t& plane = *pair.second.plane;
        
        // skip planes not relevant for TCAS
        if (!pair.second.tcas)
            continue;
        
        // Make sure plane has a slot
        GetAndReserveNextAISlotNr(plane);
        
        // still no slot?
        if (plane.multiIdx < 0 || plane.multiIdx > gMultiRef.size())
            // skip
            continue;
        // this slot taken (safety measure...should not really be needed)
        gMultiRef[plane.multiIdx].bSlotTaken = true;
        const multiDataRefsTy& mdr = gMultiRef[plane.multiIdx];

        // HACK to reduce jitter in external camera applications:
        // The camera app's callback to retrieve camera position is called
        // _before_ any drawing happens. So what we do here is to provide
        // the _next_ position for the camera callback to retrieve before
        // the _next_ cycle.
        // This is one frame off of what we are drawing. Still _very_ close ;)
        double aiX, aiY, aiZ;
        XPLMWorldToLocal(plane.nextPos.lat,
                         plane.nextPos.lon,
                         plane.nextPos.elevation * kFtToMeters,
                         &aiX, &aiY, &aiZ);
        XPLMSetDataf(mdr.X,      (float)aiX);
        XPLMSetDataf(mdr.Y,      (float)aiY);
        XPLMSetDataf(mdr.Z,      (float)aiZ);
        // attitude
        XPLMSetDataf(mdr.pitch,  plane.nextPos.pitch);
        XPLMSetDataf(mdr.roll,   plane.nextPos.roll);
        XPLMSetDataf(mdr.heading,plane.nextPos.heading);
        // configuration
        float arrGear[10] = {
            plane.surface.gearPosition, plane.surface.gearPosition, plane.surface.gearPosition,
            plane.surface.gearPosition, plane.surface.gearPosition, plane.surface.gearPosition,
            plane.surface.gearPosition, plane.surface.gearPosition, plane.surface.gearPosition,
            plane.surface.gearPosition };
        XPLMSetDatavf(mdr.gear, arrGear, 0, 10);
        XPLMSetDataf(mdr.flap,  plane.surface.flapRatio);
        XPLMSetDataf(mdr.flap2, plane.surface.flapRatio);
        // [...]
        XPLMSetDataf(mdr.yoke_pitch,    plane.surface.yokePitch);
        XPLMSetDataf(mdr.yoke_roll,     plane.surface.yokeRoll);
        XPLMSetDataf(mdr.yoke_yaw,      plane.surface.yokeHeading);
        
        // For performance reasons and because differences (cartesian velocity)
        // are smoother if calculated over "longer" time frames,
        // the following updates are done about every second only
        if (tNow - plane.prev_ts > std::chrono::seconds(1))
        {
            // do we have any prev x/y/z values at all?
            if (plane.prev_ts != std::chrono::steady_clock::time_point()) {
                // yes, so we can calculate velocity
                const std::chrono::duration<double, std::milli> dur_ms = tNow - plane.prev_ts;
                const double d_s = dur_ms.count() / 1000.0;     // seconds with fractions
                XPLMSetDataf(mdr.v_x, float((aiX - plane.prev_x) / d_s));
                XPLMSetDataf(mdr.v_y, float((aiY - plane.prev_y) / d_s));
                XPLMSetDataf(mdr.v_z, float((aiZ - plane.prev_z) / d_s));
            }
            plane.prev_x = aiX;
            plane.prev_y = aiY;
            plane.prev_z = aiZ;
            plane.prev_ts = tNow;

            // configuration (cont.)
            XPLMSetDataf(mdr.spoiler,       plane.surface.spoilerRatio);
            XPLMSetDataf(mdr.speedbrake,    plane.surface.speedBrakeRatio);
            XPLMSetDataf(mdr.slat,          plane.surface.slatRatio);
            XPLMSetDataf(mdr.wingSweep,     plane.surface.wingSweep);
            float arrThrottle[8] = {
                plane.surface.thrust, plane.surface.thrust, plane.surface.thrust,
                plane.surface.thrust, plane.surface.thrust, plane.surface.thrust,
                plane.surface.thrust, plane.surface.thrust };
            XPLMSetDatavf(mdr.throttle,     arrThrottle, 0, 10);
            // lights
            XPLMSetDatai(mdr.bcnLights,     plane.surface.lights.bcnLights);
            XPLMSetDatai(mdr.landLights,    plane.surface.lights.landLights);
            XPLMSetDatai(mdr.navLights,     plane.surface.lights.navLights);
            XPLMSetDatai(mdr.strbLights,    plane.surface.lights.strbLights);
            XPLMSetDatai(mdr.taxiLights,    plane.surface.lights.taxiLights);

            // Shared data for providing textual info (see XPMPInfoTexts_t)
            XPMPInfoTexts_t info;
            info.size = sizeof(info);
            if (XPMPGetPlaneData(&plane, xpmpDataType_InfoTexts, &info) == xpmpData_NewData)
            {
                // Note: We only update these values if there really is _new_ data!
                // Calls to XPLMSetDatab can trigger notification callbacks of listeners.
                XPLMSetDatab(mdr.infoTailNum,       info.tailNum,       0, sizeof(XPMPInfoTexts_t::tailNum));
                XPLMSetDatab(mdr.infoIcaoAcType,    info.icaoAcType,    0, sizeof(XPMPInfoTexts_t::icaoAcType));
                XPLMSetDatab(mdr.infoManufacturer,  info.manufacturer,  0, sizeof(XPMPInfoTexts_t::manufacturer));
                XPLMSetDatab(mdr.infoModel,         info.model,         0, sizeof(XPMPInfoTexts_t::model));
                XPLMSetDatab(mdr.infoIcaoAirline,   info.icaoAirline,   0, sizeof(XPMPInfoTexts_t::icaoAirline));
                XPLMSetDatab(mdr.infoAirline,       info.airline,       0, sizeof(XPMPInfoTexts_t::airline));
                XPLMSetDatab(mdr.infoFlightNum,     info.flightNum,     0, sizeof(XPMPInfoTexts_t::flightNum));
                XPLMSetDatab(mdr.infoAptFrom,       info.aptFrom,       0, sizeof(XPMPInfoTexts_t::aptFrom));
                XPLMSetDatab(mdr.infoAptTo,         info.aptTo,         0, sizeof(XPMPInfoTexts_t::aptTo));
            }
        }
        
        // remember the highest idx used
        if (plane.multiIdx > maxMultiIdxUsed)
            maxMultiIdxUsed = plane.multiIdx;
        
        // count number of multiplayer planes reported...we can stop early if
        // all slots have been filled
        if (++multiCount >= gMultiRef.size())
            break;
    }

    // Cleanup unused multiplayer datarefs
    for (multiDataRefsTy& mdr : gMultiRef)
        // if not used reset all values
        if (!mdr.bSlotTaken)
            XPMPClearMultiplayerDataRefs(mdr);

    // Final hack - leave a note to ourselves for how many of Austin's planes we relocated to do TCAS.
    gEnableCount = (maxMultiIdxUsed+1);
}

// Main function for rendering planes

void			XPMPDefaultPlaneRenderer(int is_blend)
{
	long	planeCount = XPMPCountPlanes();
#if DEBUG_RENDERER
	char	buf[50];
	sprintf(buf,"Renderer Planes: %d\n", planeCount);
	XPLMDebugString(buf);
#endif
	if (planeCount == 0)		// Quick exit if no one's around.
	{
        // make sure multiplayer dataRefs are cleaned
        XPMPInitMultiplayerDataRefs();
        
		if (gDumpOneRenderCycle)
		{
			gDumpOneRenderCycle = false;
			XPLMDebugString("No planes this cycle.\n");
		}
		return;
	}

	if (!gMSAAHackInitialised) {
		gMSAAHackInitialised = true;
		gMSAAXRatioRef = XPLMFindDataRef("sim/private/controls/hdr/fsaa_ratio_x");
		gMSAAYRatioRef = XPLMFindDataRef("sim/private/controls/hdr/fsaa_ratio_y");
        gHDROnRef      = XPLMFindDataRef("sim/graphics/settings/HDR_on");
	}

	cull_info_t			gl_camera;
	setup_cull_info(&gl_camera);
	XPLMCameraPosition_t x_camera;

	XPLMReadCameraPosition(&x_camera);	// only for zoom!

	// Culling - read the camera pos«and figure out what's visible.

	double	maxDist = XPLMGetDataf(gVisDataRef);
	double  labelDist = min(maxDist, MAX_LABEL_DIST) * x_camera.zoom;		// Labels get easier to see when users zooms.
	double	fullPlaneDist = x_camera.zoom * (5280.0 / 3.2) * (gFloatPrefsFunc ? gFloatPrefsFunc("planes","full_distance", 3.0) : 3.0);	// Only draw planes fully within 3 miles.
	int		maxFullPlanes = gIntPrefsFunc ? gIntPrefsFunc("planes","max_full_count", 100) : 100;						// Draw no more than 100 full planes!

	gTotPlanes = (int)planeCount;
	gNavPlanes = gACFPlanes = gOBJPlanes = 0;

    // Planes - sorted by (prio x distance) so we can do the closest N and bail
    //
	RenderMap						myPlanes;
	
	/************************************************************************************
	 * CULLING AND STATE CALCULATION LOOP
	 ************************************************************************************/
	
	if (gDumpOneRenderCycle)
	{
        int modelCount, active, plugin;
        XPLMCountAircraft(&modelCount, &active, &plugin);
		XPLMDebugString("Dumping one cycle map of planes.\n");
		char	fname[256], bigbuf[1024], foo[32];
		for (int n = 1; n < modelCount; ++n)
		{
			XPLMGetNthAircraftModel(n, fname, bigbuf);
			sprintf(foo, " [%d] - ", n);
			XPLMDebugString(foo);
			XPLMDebugString(fname);
			XPLMDebugString(" - ");
			XPLMDebugString(bigbuf);
			XPLMDebugString("\n");
		}
	}
	
	// Go through every plane.  We're going to figure out if it is visible and if so remember it for drawing later.
	for (long index = 0; index < planeCount; ++index)
	{
		XPMPPlanePtr id = static_cast<XPMPPlanePtr>(XPMPGetNthPlane(index));
		
		XPMPPlanePosition_t	pos;
		pos.size = sizeof(pos);
		pos.label[0] = 0;

		if (XPMPGetPlaneData(id, xpmpDataType_Position, &pos) != xpmpData_Unavailable)
		{
			// First figure out where the plane is!

			double	x,y,z;
			XPLMWorldToLocal(pos.lat, pos.lon, pos.elevation * kFtToMeters, &x, &y, &z);
			
			float distMeters = sqrtf(sphere_distance_sqr(&gl_camera,
														 static_cast<float>(x),
														 static_cast<float>(y),
														 static_cast<float>(z)));
			
			// Only draw if it's in range.
			bool cull = (distMeters > maxDist);
			
			XPMPPlaneRadar_t radar;
			radar.size = sizeof(radar);
			bool tcas = true;
            int aiPrio = pos.aiPrio;            // instead of deciding not to show a/c we use priorities to fill up the 19 slots with the most TCAS-interesting a/c
			if (XPMPGetPlaneData(id, xpmpDataType_Radar, &radar) != xpmpData_Unavailable)
				if (radar.mode == xpmpTransponderMode_Standby)
					tcas = false;

			// check for altitude - if difference exceeds a preconfigured limit, reduce prio
			double acft_alt = XPLMGetDatad(gAltitudeRef) / kFtToMeters;
			double alt_diff = pos.elevation - acft_alt;
			if(alt_diff < 0) alt_diff *= -1;
			if(alt_diff > MAX_TCAS_ALTDIFF) aiPrio += 3;

			// Calculate the heading from the camera to the target (hor, vert).
			// Calculate the angles between the camera angles and the real angles.
			// Cull if we exceed half the FOV.
			
			if(!cull && !sphere_is_visible(&gl_camera, static_cast<float>(x),
										   static_cast<float>(y),
										   static_cast<float>(z), 50.0))
			{
				cull = true;
			}
			
			// Full plane or lites based on distance.
			bool	drawFullPlane = (distMeters < fullPlaneDist);
			
#if DEBUG_RENDERER

			char	icao[128], livery[128];
			char	debug[512];

			XPMPGetPlaneICAOAndLivery(id, icao, livery);
			sprintf(debug,"Queueing plane %d (%s/%s) at lle %f, %f, %f (xyz=%f, %f, %f) pitch=%f,roll=%f,heading=%f,model=1.\n", index, icao, livery,
					pos.lat, pos.lon, pos.elevation,
					x, y, z, pos.pitch, pos.roll, pos.heading);
			XPLMDebugString(debug);
#endif
            // Not on TCAS? Then it occupies no multiplayer idx
            if (!tcas)
                id->multiIdx = -1;

			// Stash one render record with the plane's position, etc.
			{
				PlaneToRender_t		renderRecord;
				renderRecord.x = static_cast<float>(x);
				renderRecord.y = static_cast<float>(y);
				renderRecord.z = static_cast<float>(z);
				renderRecord.plane = static_cast<XPMPPlanePtr>(id);
				renderRecord.cull = cull;						// NO other planes.  Doing so causes a lot of things to go nuts!
				renderRecord.tcas = tcas;

				XPMPPlaneSurfaces_t	surfaces;
				surfaces.size = sizeof(surfaces);
				if (XPMPGetPlaneData(id, xpmpDataType_Surfaces, &surfaces) != xpmpData_Unavailable)
				{
					renderRecord.state.structSize = sizeof(renderRecord.state);
					renderRecord.state.gearPosition 	= surfaces.gearPosition 	;
					renderRecord.state.flapRatio 		= surfaces.flapRatio 		;
					renderRecord.state.spoilerRatio 	= surfaces.spoilerRatio 	;
					renderRecord.state.speedBrakeRatio 	= surfaces.speedBrakeRatio 	;
					renderRecord.state.slatRatio 		= surfaces.slatRatio 		;
					renderRecord.state.wingSweep 		= surfaces.wingSweep 		;
					renderRecord.state.thrust 			= surfaces.thrust 			;
					renderRecord.state.yokePitch 		= surfaces.yokePitch 		;
					renderRecord.state.yokeHeading 		= surfaces.yokeHeading 		;
					renderRecord.state.yokeRoll 		= surfaces.yokeRoll 		;
				} else {
					renderRecord.state.structSize = sizeof(renderRecord.state);
					renderRecord.state.gearPosition = (pos.elevation < 70) ?  1.0f : 0.0f;
					renderRecord.state.flapRatio = (pos.elevation < 70) ? 1.0f : 0.0f;
					renderRecord.state.spoilerRatio = renderRecord.state.speedBrakeRatio = renderRecord.state.slatRatio = renderRecord.state.wingSweep = 0.0;
					renderRecord.state.thrust = (pos.pitch > 30) ? 1.0f : 0.6f;
					renderRecord.state.yokePitch = pos.pitch / 90.0f;
					renderRecord.state.yokeHeading = pos.heading / 180.0f;
					renderRecord.state.yokeRoll = pos.roll / 90.0f;

					// use some smart defaults
					renderRecord.plane->surface.lights.bcnLights = 1;
					renderRecord.plane->surface.lights.navLights = 1;
				}
				if (renderRecord.plane->model && !renderRecord.plane->model->moving_gear)
					renderRecord.plane->surface.gearPosition = 1.0;
				renderRecord.full = drawFullPlane;
				renderRecord.dist = distMeters;
                // myPlanes is sorted considering aiPrio and distance.
                // We achieve that by adding a high constant value (10nm) per prio:
				myPlanes.emplace(float(aiPrio * kAIPrioMultiplierMeters + distMeters),
                                 renderRecord);

			} // State calculation
			
		} // Plane has data available
		
	} // Per-plane loop

	if (gDumpOneRenderCycle)
		XPLMDebugString("End of cycle dump.\n");
    
	/************************************************************************************
	 * ACTUAL RENDERING LOOP
	 ************************************************************************************/
	
	// We're going to go in and render the first N planes in full, and the rest as lites.
	// We're also going to put the x-plane multiplayer vars in place for the first N
	// TCAS-visible planes, so they show up on our moving map.
	// We do this in two stages: building up what to do, then doing it in the optimal
	// OGL order.
	
	vector<PlaneToRender_t *>			planes_obj_lites;
	multimap<int, PlaneToRender_t *>	planes_austin;
	vector<PlaneToRender_t *>			planes_obj;
	vector<PlaneToRender_t *>			planes_obj8;

	vector<PlaneToRender_t *>::iterator			planeIter;
	multimap<int, PlaneToRender_t *>::iterator	planeMapIter;

	// In our first iteration pass we'll go through all planes and handle TCAS, draw planes that have no
	// CSL model, and put CSL planes in the right 'bucket'.

	for (RenderMap::iterator iter = myPlanes.begin(); iter != myPlanes.end(); ++iter)
	{
		// This is the case where we draw a real plane.
		if (!iter->second.cull)
		{
			// Max plane enforcement - once we run out of the max number of full planes the
			// user allows, force only lites for framerate
			if (gACFPlanes >= maxFullPlanes)
				iter->second.full = false;

#if DEBUG_RENDERER
			char	debug[512];
			sprintf(debug,"Drawing plane: %s at %f,%f,%f (%fx%fx%f full=%d\n",
					iter->second.plane->model ? iter->second.plane->model->file_path.c_str() : "<none>", iter->second.x, iter->second.y, iter->second.z,
					iter->second.plane->pos.pitch, iter->second.plane->pos.roll, iter->second.plane->pos.heading, iter->second.full ? 1 : 0);
			XPLMDebugString(debug);
#endif

			if (iter->second.plane->model)
			{
				// always check for the offset since we need it in multiple places.
				cslVertOffsetCalc.findOrUpdateActualVertOffset(*iter->second.plane->model);
				if (iter->second.plane->pos.offsetScale > 0.0f) {
					iter->second.y += iter->second.plane->pos.offsetScale * float(iter->second.plane->model->actualVertOffset);
				}
				if (iter->second.plane->pos.clampToGround || (gIntPrefsFunc("planes", "clamp_all_to_ground", 0) != 0)) {
					//correct y value by real terrain elevation
					//find or update the actual vert offset in the csl model data
					cslVertOffsetCalc.findOrUpdateActualVertOffset(*iter->second.plane->model);
					iter->second.y = (float)correctYValue(
						iter->second.x, iter->second.y, iter->second.z, iter->second.plane->model->actualVertOffset);
				}
				if (iter->second.plane->model->plane_type == plane_Austin)
				{
					planes_austin.insert(multimap<int, PlaneToRender_t *>::value_type(CSL_GetOGLIndex(iter->second.plane->model), &iter->second));
				}
				else if (iter->second.plane->model->plane_type == plane_Obj)
				{
					planes_obj.push_back(&iter->second);
					planes_obj_lites.push_back(&iter->second);
				}
				else if(iter->second.plane->model->plane_type == plane_Obj8)
				{
					planes_obj8.push_back(&iter->second);
				}

			} else {
				// If it's time to draw austin's planes but this one
				// doesn't have a model, we draw anything.
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glTranslatef(iter->second.x, iter->second.y, iter->second.z);
				glRotatef(iter->second.plane->pos.heading, 0.0, -1.0, 0.0);
				glRotatef(iter->second.plane->pos.pitch, 01.0, 0.0, 0.0);
				glRotatef(iter->second.plane->pos.roll, 0.0, 0.0, -1.0);

				// Safety check - if plane 1 isn't even loaded do NOT draw, do NOT draw plane 0.
				// Using the user's planes can cause the internal flight model to get f-cked up.
				// Using a non-loaded plane can trigger internal asserts in x-plane.
                int modelCount, active, plugin;
                XPLMCountAircraft(&modelCount, &active, &plugin);
				if (modelCount > 1)
					if(!is_blend)
						XPLMDrawAircraft(1,
										 (float) iter->second.x, (float) iter->second.y, (float) iter->second.z,
										 iter->second.plane->pos.pitch, iter->second.plane->pos.roll, iter->second.plane->pos.heading,
										 iter->second.full ? 1 : 0, &iter->second.state);

				glPopMatrix();
			}

		}

	}
	
	// PASS 1 - draw Austin's planes.
	if(gHasControlOfAIAircraft && !is_blend)
		for (planeMapIter = planes_austin.begin(); planeMapIter != planes_austin.end(); ++planeMapIter)
		{
			CSL_DrawObject(	planeMapIter->second->plane,
							planeMapIter->second->dist,
							planeMapIter->second->x,
							planeMapIter->second->y,
							planeMapIter->second->z,
							planeMapIter->second->plane->pos.pitch,
							planeMapIter->second->plane->pos.roll,
							planeMapIter->second->plane->pos.heading,
							plane_Austin,
							planeMapIter->second->full ? 1 : 0,
							planeMapIter->second->plane->surface,
							&planeMapIter->second->state);

			if (planeMapIter->second->full)
				++gACFPlanes;
			else
				++gNavPlanes;
		}
	
	// PASS 2 - draw OBJs
	// Blend for solid OBJ7s?  YES!  First, in HDR mode, they DO NOT draw to the gbuffer properly -
	// they splat their livery into the normal map, which is terrifying and stupid.  Then they are also
	// pre-lit...the net result is surprisingly not much worse than regular rendering considering how many
	// bad things have happened, but for all I know we're getting NaNs somewhere.
	//
	// Blending isn't going to hurt things in NON-HDR because our rendering is so stupid for old objs - there's
	// pretty much never translucency so we aren't going to get Z-order fails.  So f--- it...always draw blend.<
	if(is_blend)
		for (const auto &plane_obj : planes_obj)
		{
			CSL_DrawObject(
						plane_obj->plane,
						plane_obj->dist,
						plane_obj->x,
						plane_obj->y,
						plane_obj->z,
						plane_obj->plane->pos.pitch,
						plane_obj->plane->pos.roll,
						plane_obj->plane->pos.heading,
						plane_Obj,
						plane_obj->full ? 1 : 0,
						plane_obj->plane->surface,
						&plane_obj->state);
			++gOBJPlanes;
		}

	for(planeIter = planes_obj8.begin(); planeIter != planes_obj8.end(); ++planeIter)
	{
		CSL_DrawObject( (*planeIter)->plane,
						(*planeIter)->dist,
						(*planeIter)->x,
						(*planeIter)->y,
						(*planeIter)->z,
						(*planeIter)->plane->pos.pitch,
						(*planeIter)->plane->pos.roll,
						(*planeIter)->plane->pos.heading,
						plane_Obj8,
						(*planeIter)->full ? 1 : 0,
						(*planeIter)->plane->surface,
						&(*planeIter)->state);
	}

	if(!is_blend)
		obj_draw_solid();

	// PASS 3 - draw OBJ lights.

	if(is_blend)
		if (!planes_obj_lites.empty())
		{
			OBJ_BeginLightDrawing();
			for (planeIter = planes_obj_lites.begin(); planeIter != planes_obj_lites.end(); ++planeIter)
			{
				// this thing draws the lights of a model
				CSL_DrawObject( (*planeIter)->plane,
								(*planeIter)->dist,
								(*planeIter)->x,
								(*planeIter)->y,
								(*planeIter)->z,
								(*planeIter)->plane->pos.pitch,
								(*planeIter)->plane->pos.roll,
								(*planeIter)->plane->pos.heading,
								plane_Lights,
								(*planeIter)->full ? 1 : 0,
								(*planeIter)->plane->surface,
								&(*planeIter)->state);
			}
		}
	
	obj_draw_done();
	
	// PASS 4 - Labels
	if(is_blend)
		if ( gDrawLabels )
		{
			double	x_scale = 1.0;
			double	y_scale = 1.0;
            if (gHDROnRef && XPLMGetDatai(gHDROnRef)) {     // SSAA hack only if HDR enabled
                if (gMSAAXRatioRef) {
                    x_scale = XPLMGetDataf(gMSAAXRatioRef);
                }
                if (gMSAAYRatioRef) {
                    y_scale = XPLMGetDataf(gMSAAYRatioRef);
                }
            }

			GLint	vp[4];
			if (viewportRef != nullptr) {
				// sim/graphics/view/viewport	int[4]	n	Pixels	Current OpenGL viewport in device window coordinates.Note thiat this is left, bottom, right top, NOT left, bottom, width, height!!
				int vpInt[4] = {0,0,0,0};
				XPLMGetDatavi(viewportRef, vpInt, 0, 4);
				vp[0] = vpInt[0];
				vp[1] = vpInt[1];
				vp[2] = vpInt[2] - vpInt[0];
				vp[3] = vpInt[3] - vpInt[1];
			} else {		
				glGetIntegerv(GL_VIEWPORT, vp);
			}

			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glOrtho(0, vp[2], 0, vp[3], -1, 1);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			if (x_scale > 1.0 || y_scale > 1.0) {
				glScalef((GLfloat)x_scale, (GLfloat)y_scale, 1.0f);
			} else {
				x_scale = 1.0;
				y_scale = 1.0;
			}

			for (RenderMap::iterator iter = myPlanes.begin(); iter != myPlanes.end(); ++iter)
				if(iter->second.dist < labelDist)
					if(!iter->second.cull)		// IMPORTANT - airplane BEHIND us still maps XY onto screen...so we get 180 degree reflections.  But behind us acf are culled, so that's good.
					{
						float x, y;
						convert_to_2d(&gl_camera, vp, iter->second.x, iter->second.y, iter->second.z, 1.0, &x, &y);

                        // base color can be defined per plane
                        // rat is between 0.0 (plane very close) and 1.0 (shortly before label cut-off):
                        // and defines how much we move towards light gray for distance
                        const PlaneToRender_t& ptr = iter->second;
                        const float rat = iter->second.dist / static_cast<float>(labelDist);
                        constexpr float gray[4] = {0.6f, 0.6f, 0.6f, 1.0f};
                        float c[4] = {
                            (1.0f-rat) * ptr.plane->pos.label_color[0] + rat * gray[0],     // red
                            (1.0f-rat) * ptr.plane->pos.label_color[1] + rat * gray[1],     // green
                            (1.0f-rat) * ptr.plane->pos.label_color[2] + rat * gray[2],     // blue
                            (1.0f-rat) * ptr.plane->pos.label_color[3] + rat * gray[3]      // ? (not used for text)
                        };

						XPLMDrawString(c, static_cast<int>(x / x_scale), static_cast<int>(y / y_scale)+10, (char *) iter->second.plane->pos.label, NULL, xplmFont_Basic);
					}

			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();

		}

    // Handling of TCAS/AI/Multiplayer DataRefs
    XPMPMultiplayerHandling(myPlanes);
	
	gDumpOneRenderCycle = 0;

	// finally, cleanup textures.
	OBJ_MaintainTextures();
}

void XPMPEnableAircraftLabels()
{
	gDrawLabels = true;
}

void XPMPDisableAircraftLabels()
{
	gDrawLabels = false;
}

bool XPMPDrawingAircraftLabels()
{
	return gDrawLabels;
}

