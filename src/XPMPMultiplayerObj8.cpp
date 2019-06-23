/* 
 * Copyright (c) 2013, Laminar Research.
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

#include "XPMPMultiplayerObj8.h"
#include "XPMPMultiplayerVars.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include <stddef.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>
using namespace std;

struct	one_inst {
	one_inst * next;
	
	XPLMDrawInfo_t			location;
	XPMPPlaneSurfaces_t		surface;
	XPLMPlaneDrawState_t *	state;
	
	~one_inst() { delete next; }
};

struct	one_obj {
	one_obj *			next;
	obj_for_acf *		model;
	one_inst *			head;
	
	~one_obj() { delete head; delete next; }
};

static	one_obj *	s_worklist = NULL;


static one_inst *	s_cur_plane = NULL;

enum DR_VALUES {
	gear_rat = 0,
	flap_rat,
	spoi_rat,
	sbrk_rat,
	slat_rat,
	swep_rat,
	thrs_rat,
	ptch_rat,
	head_rat,
	roll_rat,
	thrs_rev,

	tax_lite_on,
	lan_lite_on,
	bcn_lite_on,
	str_lite_on,
	nav_lite_on,
    
    engine_rot_angle,
    engine_rot_speed_rpm,
    engine_rot_speed_rad,
    prop_rot_angle,
    prop_rot_speed_rpm,
    prop_rot_speed_rad,
    revers_rat,
    
    touch_down,
	
	dref_dim
};

// dataRef names are missing the root name. This will be
// 1. always the text defined by XPMP_CLIENT_NAME, so that the dataRefs
//    offered to CSL models are unique per application, but certainly require
//    the models to access this unique dataRef name, wich usually will mean
//    that their .obj files have to modified like CSL2XSB is supposed to offer it.
// 2. _optionally_ _additonally_ the standard "libxplanemp" text, compatible with
//    usual versions of CSL models but causing conflicts if several multiplayer clients
//    are running in parallel. This is why experiences users may want to switch
//    this backward compatibility off.
// The application controls if "libxplanemp" is registered by the
// integer config option "planes"/"dr_libxplanemp".
const char * DREF_STD_ROOT = "libxplanemp";
const char * dref_names[dref_dim] = {
	"/controls/gear_ratio",
	"/controls/flap_ratio",
	"/controls/spoiler_ratio",
	"/controls/speed_brake_ratio",
	"/controls/slat_ratio",
	"/controls/wing_sweep_ratio",
	"/controls/thrust_ratio",
	"/controls/yoke_pitch_ratio",
	"/controls/yoke_heading_ratio",
	"/controls/yoke_roll_ratio",
	"/controls/thrust_revers",

	"/controls/taxi_lites_on",
	"/controls/landing_lites_on",
	"/controls/beacon_lites_on",
	"/controls/strobe_lites_on",
	"/controls/nav_lites_on",
    
    "/engines/engine_rotation_angle_deg",
    "/engines/engine_rotation_speed_rpm",
    "/engines/engine_rotation_speed_rad_sec",       // PE defines this: https://www.pilotedge.net/pages/csl-authoring
    "/engines/prop_rotation_angle_deg",
    "/engines/prop_rotation_speed_rpm",
    "/engines/prop_rotation_speed_rad_sec",
    "/engines/thrust_reverser_deploy_ratio",
    
    "/misc/touch_down"
};


static float obj_get_float(void * inRefcon)
{
	if(s_cur_plane == NULL) return 0.0f;
	
	DR_VALUES v = (DR_VALUES)reinterpret_cast<intptr_t>(inRefcon);
	switch(v)
	{
	case gear_rat:			return s_cur_plane->state->gearPosition;		break;
	case flap_rat:			return s_cur_plane->state->flapRatio;			break;
	case spoi_rat:			return s_cur_plane->state->spoilerRatio;		break;
	case sbrk_rat:			return s_cur_plane->state->speedBrakeRatio;		break;
	case slat_rat:			return s_cur_plane->state->slatRatio;			break;
	case swep_rat:			return s_cur_plane->state->wingSweep;			break;
	case thrs_rat:			return s_cur_plane->state->thrust;				break;
	case ptch_rat:			return s_cur_plane->state->yokePitch;			break;
	case head_rat:			return s_cur_plane->state->yokeHeading;			break;
	case roll_rat:			return s_cur_plane->state->yokeRoll;			break;
	case thrs_rev:			return (s_cur_plane->state->thrust < 0.0f) ? 1.0f : 0.0f; break; //if thrust less than zero, reverse is on

	case tax_lite_on:		return static_cast<float>(s_cur_plane->surface.lights.taxiLights);
	case lan_lite_on:		return static_cast<float>(s_cur_plane->surface.lights.landLights);
	case bcn_lite_on:		return static_cast<float>(s_cur_plane->surface.lights.bcnLights);
	case str_lite_on:		return static_cast<float>(s_cur_plane->surface.lights.strbLights);
	case nav_lite_on:		return static_cast<float>(s_cur_plane->surface.lights.navLights);

    case engine_rot_angle:      return s_cur_plane->surface.engRotDegree;
    case engine_rot_speed_rpm:  return s_cur_plane->surface.engRotRpm;
    case engine_rot_speed_rad:  return s_cur_plane->surface.engRotRpm * 0.10471975511966f;
    case prop_rot_angle:        return s_cur_plane->surface.propRotDegree;
    case prop_rot_speed_rpm:    return s_cur_plane->surface.propRotRpm;
    case prop_rot_speed_rad:    return s_cur_plane->surface.propRotRpm * 0.10471975511966f;
    case revers_rat:            return s_cur_plane->surface.reversRatio;
            
    case touch_down:            return s_cur_plane->surface.touchDown ? 1.0f : 0.0f;
            
	default:
		return 0.0f;
	}
}

int obj_get_float_array(
		void *               inRefcon,
		float *              inValues,
		int                  /*inOffset*/,
		int                  inCount)
{
	if(inValues == NULL)
		return 1;
	float rv = obj_get_float(inRefcon);
	for(int i = 0; i < inCount; ++i)
		inValues[i] = rv;
	return inCount;
}

bool obj8_load_async = true;

void	obj_init()
{
	int sim, xplm;
	XPLMHostApplicationID app;
	XPLMGetVersions(&sim,&xplm,&app);
	// Ben says: we need the 2.10 SDK (e.g. X-Plane 10) to have async load at all.  But we need 10.30 to pick up an SDK bug
	// fix where async load crashes if we queue a second load before the first completes.  So for users on 10.25, they get
	// pauses.
	if (1 == gIntPrefsFunc("debug", "allow_obj8_async_load", 0) && sim >= 10300) {
		obj8_load_async = true;	
	} else {
		obj8_load_async = false;
	}
	
    // Shall we register only the client specific dataRefs,
    // or for compatibility also the standard name "libxplanemp"? (default: yes!)
    bool bRegisterLibMP = gIntPrefsFunc("planes", "dr_libxplanemp", 1) != 0;
    
    // always register the client-specific dataRefs
    // only if requested register the standard libxplanemp dataRefs
    for (std::string drRoot: { XPMP_CLIENT_NAME, DREF_STD_ROOT })
    {
        for(int i = 0; i < dref_dim; ++i)
        {
            XPLMRegisterDataAccessor(
                        (drRoot + dref_names[i]).c_str(),
                        xplmType_Float|xplmType_FloatArray, 0,
                        NULL, NULL,
                        obj_get_float, NULL,
                        NULL, NULL,
                        NULL, NULL,
                        obj_get_float_array, NULL,
                        NULL, NULL, reinterpret_cast<void *>(static_cast<intptr_t>(i)), NULL);
        }
        
        // exit loop if we shall not init the standard dataRef
        if (!bRegisterLibMP)
            break;
    }
}

void obj_loaded_cb(XPLMObjectRef obj, void * refcon)
{
	auto *model = reinterpret_cast<obj_for_acf *>(refcon);
	if (obj != nullptr) {
		model->load_state = load_loaded;
		model->handle = obj;
#ifdef DEBUG
		XPLMDebugString(XPMP_CLIENT_NAME": Notified Successful Async Load of OBJ8: ");
		XPLMDebugString(model->file.c_str());
		XPLMDebugString("\n");
#endif
	} else {
		model->load_state = load_failed;
#ifdef DEBUG
		XPLMDebugString(XPMP_CLIENT_NAME": Notified Failed Async Load of OBJ8: ");
		XPLMDebugString(model->file.c_str());
		XPLMDebugString("\n");
#endif
	}
} 

void	obj_schedule_one_aircraft(
		CSLPlane_t *			model,
		double 					x,
		double 					y,
		double 					z,
		double 					pitch,
		double 					roll,
		double 					heading,
		int	   					/*full*/,		//
		XPMPPlaneSurfaces_t		surface,
		XPLMPlaneDrawState_t *	state)
{
	one_obj * iter;
	
	for(auto att = model->attachments.begin(); att != model->attachments.end(); ++att)
	{
		obj_for_acf * obj8 = &*att;

		if(obj8->handle == nullptr && obj8->load_state == load_none && !obj8->file.empty()) {
#ifdef DEBUG
			XPLMDebugString(XPMP_CLIENT_NAME ": Loading Model ");
			XPLMDebugString(obj8->file.c_str());
			XPLMDebugString("\n");
#endif			
			if(obj8_load_async) {
				XPLMLoadObjectAsync(obj8->file.c_str(),obj_loaded_cb,reinterpret_cast<void *>(obj8));
				obj8->load_state = load_loading;
			} else {
				obj8->handle = XPLMLoadObject(obj8->file.c_str());
				if (obj8->handle != nullptr) {
					obj8->load_state = load_loaded;
				} else {
					obj8->load_state = load_failed;
				}
			}
		}

		for(iter = s_worklist; iter; iter = iter->next)
		{
			if(iter->model == obj8)
				break;
		}
		if(iter == nullptr)
		{
			iter = new one_obj;
			iter->next = s_worklist;
			s_worklist = iter;
			iter->model = obj8;
			iter->head = nullptr;
		}
		
		if(iter->model->load_state == load_loaded) {
			auto * i = new one_inst;
			i->next = iter->head;
			iter->head = i;
			i->surface = surface;
			i->state = state;
			i->location.structSize = sizeof(i->location);
			i->location.x = static_cast<float>(x);
			i->location.y = static_cast<float>(y);
			i->location.z = static_cast<float>(z);
			i->location.pitch = static_cast<float>(pitch);
			i->location.roll = static_cast<float>(roll);
			i->location.heading = static_cast<float>(heading);
		}
	}
}

void	obj_draw_solid()
{
	one_obj *who = s_worklist;
	while(who)
	{
		static XPLMDataRef night_lighting_ref = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
		bool use_night = XPLMGetDataf(night_lighting_ref) > 0.25;

		for (one_inst * i = who->head; i; i = i->next)
		{
			s_cur_plane = i;
			// TODO: set obj sate to state datarefs(dref_names) from "one_inst".
			XPLMDrawObjects(who->model->handle, 1, &i->location, use_night, 0);
		}
		who = who->next;
	}
	s_cur_plane = nullptr;
}

void	obj_draw_done()
{
	delete s_worklist;
	s_worklist = nullptr;
}
