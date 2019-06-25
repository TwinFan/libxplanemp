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

#include "XPMPMultiplayer.h"
#include "XPMPMultiplayerVars.h"
#include "XPMPPlaneRenderer.h"
#include "XPMPMultiplayerCSL.h"
#include "XPMPMultiplayerCSLOffset.h"
#include "XPLMUtilities.h"

#include <algorithm>
#include <cctype>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>

#include "XPLMProcessing.h"
#include "XPLMPlanes.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

#include "XOGLUtils.h"
//#include "PlatformUtils.h"

#include <stdlib.h>
#include <stdio.h>
#include <set>
#include <cassert>

// This prints debug info on our process of loading Austin's planes.
#define	DEBUG_MANUAL_LOADING	0


/******************************************************************************

	T H E   T C A S   H A C K
	
 The 1.0 SDK provides no way to add TCAS blips to a panel - we simply don't know
 where on the panel to draw.  The only way to get said blips is to manipulate
 Austin's "9 planes" plane objects, which he refers to when drawing the moving
 map.
 
 But how do we integrate this with our system, which relies on us doing
 the drawing (either by calling Austin's low level drawing routine or just
 doing it ourselves with OpenGL)?
 
 The answer is the TCAS hack.  Basically we set Austin's number of multiplayer
 planes to zero while 3-d drawing is happening so he doesn't draw.  Then during
 2-d drawing we pop this number back up to the number of planes that are
 visible on TCAS and set the datarefs to move them, so that they appear on TCAS.

 One note: since all TCAS blips are the same, we do no model matching for
 TCAS - we just place the first 9 planes at the right place.
 
 Our rendering loop records for us in gEnableCount how many TCAS planes should
 be visible.

 ******************************************************************************/


/* we can only manipulate the TCAS, etc, if we have control of the AI aircraft
 *
 * Non-static because this information is actually needed in the renderer, but
 * we don't want it known outside of libxplanemp and both this and the renderer's
 * headers are public!  argh!
*/
bool	gHasControlOfAIAircraft = false;

static	XPMPPlanePtr	XPMPPlaneFromID(
		XPMPPlaneID 		inID,
		XPMPPlaneVector::iterator * outIter = nullptr);

// This drawing hook is called once per frame to do the real drawing.
static	int				XPMPRenderMultiplayerPlanes(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

// This drawing hook is called twice per frame to control how many planes
// should be visible.
static	int				XPMPControlPlaneCount(
		XPLMDrawingPhase     inPhase,
		int                  inIsBefore,
		void *               inRefcon);

void actualVertOffsetInfo(const char *inMtl, char *outType, double *outOffset) {
	std::string type;
	double offset;
	cslVertOffsetCalc.actualVertOffsetInfo(inMtl, type, offset);
	if(outType) {
		std::strcpy(outType, type.c_str());
	}
	if (outOffset) {
		*outOffset = offset;
	}
}

void setUserVertOffset(const char *inMtlCode, double inOffset) {
	cslVertOffsetCalc.setUserVertOffset(inMtlCode, inOffset);
}

void removeUserVertOffset(const char *inMtlCode) {
	cslVertOffsetCalc.removeUserVertOffset(inMtlCode);
}

#ifdef DEBUG_GL
static void xpmpKhrDebugProc(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *param)
{
	std::stringstream	msgOut;
	msgOut << XPMP_CLIENT_NAME << ": GL Debug: ";
	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR:
		msgOut << "[ERR] ";
		break;
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
		msgOut << "[DEP] ";
		break;
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
		msgOut << "[UND] ";
		break;
	case GL_DEBUG_TYPE_PORTABILITY:
		msgOut << "[PORT] ";
		break;
	case GL_DEBUG_TYPE_PERFORMANCE:
		msgOut << "[PERF] ";
		break;
	case GL_DEBUG_TYPE_MARKER:
		msgOut << "*** ";
		break;
	case GL_DEBUG_TYPE_PUSH_GROUP:
		msgOut << " -> ";
		break;
	case GL_DEBUG_TYPE_POP_GROUP:
		msgOut << " <- ";
		break;
	default:
		break;
	}
	msgOut << std::string(message, length) << std::endl;
	XPLMDebugString(msgOut.str().c_str());
}

void XPMPSetupGLDebug()
{
	glDebugMessageCallback(xpmpKhrDebugProc, NULL);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_FALSE);

	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_ERROR, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR, GL_DONT_CARE, 0, NULL, GL_TRUE);

	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DONT_CARE, GL_DEBUG_SEVERITY_HIGH, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DONT_CARE, GL_DEBUG_SEVERITY_MEDIUM, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_PUSH_GROUP, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_POP_GROUP, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, GL_DONT_CARE, 0, NULL, GL_TRUE);
	glDebugMessageControl(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, GL_TRUE);
	// eat errors.
	while (GL_NO_ERROR != glGetError())
		;
}
#endif

/********************************************************************************
 * SETUP
 ********************************************************************************/


const char * 	XPMPMultiplayerInitLegacyData(
		const char * inCSLFolder, const char * inRelatedPath,
		const char * inTexturePath, const char * inDoc8643,
		const char * inDefaultPlane,
		int (* inIntPrefsFunc)(const char *, const char *, int),
		float (* inFloatPrefsFunc)(const char *, const char *, float))
{
	gDefaultPlane = inDefaultPlane;
	gIntPrefsFunc = inIntPrefsFunc;
	gFloatPrefsFunc = inFloatPrefsFunc;

	// Set up OpenGL for our drawing callbacks
	OGL_UtilsInit();

#ifdef DEBUG_GL
	XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: This build includes OpenGL Debugging\n");
	XPLMDebugString("    OpenGL Debugging induces a large overhead and produces large logfiles.\n");
	XPLMDebugString("    Please do not use this build other than as directed.\n");
#endif

	OGLDEBUG(XPLMDebugString(XPMP_CLIENT_NAME " - GL supports debugging\n"));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS));
	OGLDEBUG(XPMPSetupGLDebug());
	
	xpmp_tex_useAnisotropy = OGL_HasExtension("GL_EXT_texture_filter_anisotropic");
	if (xpmp_tex_useAnisotropy) {
		GLfloat maxAnisoLevel;

		XPLMDebugString(XPMP_CLIENT_NAME " - GL supports anisoptropic filtering.\n");

		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisoLevel);
		xpmp_tex_maxAnisotropy = maxAnisoLevel;
	}
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &xpmp_tex_maxSize);

	bool	problem = false;
	if (!CSL_LoadCSL(inCSLFolder, inRelatedPath, inDoc8643))
		problem = true;

	if (!CSL_Init(inTexturePath))
		problem = true;

	if (problem)		return "There were problems initializing " XPMP_CLIENT_LONGNAME ". Please examine X-Plane's Log.txt file for detailed information.";
	else 				return "";
}

const char *    XPMPMultiplayerOBJ7SupportEnable(const char * inTexturePath) {
	// Set up OpenGL for our drawing callbacks
	OGL_UtilsInit();
#ifdef DEBUG_GL
	XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: This build includes OpenGL Debugging\n");
	XPLMDebugString("    OpenGL Debugging induces a large overhead and produces large logfiles.\n");
	XPLMDebugString("    Please do not use this build other than as directed.\n");
#endif

	OGLDEBUG(XPLMDebugString(XPMP_CLIENT_NAME " - GL supports debugging\n"));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT));
	OGLDEBUG(glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS));
	OGLDEBUG(XPMPSetupGLDebug());
	
	xpmp_tex_useAnisotropy = OGL_HasExtension("GL_EXT_texture_filter_anisotropic");
	if (xpmp_tex_useAnisotropy) {
		GLfloat maxAnisoLevel;
		
		XPLMDebugString(XPMP_CLIENT_NAME " - GL supports anisoptropic filtering.\n");
		
		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisoLevel);
		xpmp_tex_maxAnisotropy = maxAnisoLevel;
	}
	glGetIntegerv(GL_MAX_TEXTURE_SIZE, &xpmp_tex_maxSize);

	bool problem = false;
	if (!CSL_Init(inTexturePath))
		problem = true;

	if (problem) return "There was a problem initializing " XPMP_CLIENT_LONGNAME ". Please examine X-Plane's Log.txt file for detailed information.";
	else         return "";
}

const char * 	XPMPMultiplayerInit(
		int (* inIntPrefsFunc)(const char *, const char *, int),
		float (* inFloatPrefsFunc)(const char *, const char *, float),
		const char * resourceDir)
{
	gIntPrefsFunc = inIntPrefsFunc;
	gFloatPrefsFunc = inFloatPrefsFunc;
	//char	myPath[1024];
	//char	airPath[1024];
	//char	line[256];
	//char	sysPath[1024];
	//FILE *	fi;

	cslVertOffsetCalc.setResourcesDir(resourceDir);
	
	bool	problem = false;

	XPMPInitDefaultPlaneRenderer();

	// Register the actual drawing func.
	if ( !XPLMRegisterDrawCallback(XPMPRenderMultiplayerPlanes,
							 xplm_Phase_Airplanes, 0, /* after*/ 0 /* refcon */))
        problem=true;

	if (problem)		return "There were problems initializing " XPMP_CLIENT_LONGNAME ".  Please examine X-Plane's error.out file for detailed information.";
	else 				return "";
}

void XPMPMultiplayerCleanup(void)
{
    XPLMUnregisterDrawCallback(XPMPRenderMultiplayerPlanes, xplm_Phase_Airplanes, 0, 0);
	XPMPDeinitDefaultPlaneRenderer();
	OGLDEBUG(glDebugMessageCallback(NULL, NULL));
}


// We use this array to track Austin's planes, since we have to mess with them.
static	vector<string>	gPlanePaths;

const  char * XPMPMultiplayerEnable(void)
{
	// First build up a list of all of Austin's planes, and assign
	// their effective index numbers.
	gPlanePaths.clear();
	std::vector<char *>		ptrs;
	gPlanePaths.push_back("");
	
	for (size_t p = 0; p < gPackages.size(); ++p)
	{
		for (size_t pp = 0; pp < gPackages[p].planes.size(); ++pp)
		{
			if (gPackages[p].planes[pp].plane_type == plane_Austin)
			{
				gPackages[p].planes[pp].austin_idx = static_cast<int>(gPlanePaths.size());
				char	buf[1024];
				strcpy(buf,gPackages[p].planes[pp].file_path.c_str());
#if APL
				if (XPLMIsFeatureEnabled("XPLM_USE_NATIVE_PATHS") == 0)
				{
					Posix2HFSPath(buf, buf, 1024);
				}
#endif
				gPlanePaths.push_back(buf);
			}
		}
	}
	
	// Copy the list into something that's not permanent, but is needed by the XPLM.
	for (size_t n = 0; n < gPlanePaths.size(); ++n)
	{
#if DEBUG_MANUAL_LOADING
		char	strbuf[1024];
		sprintf(strbuf, "Plane %d = '%s'\n", static_cast<int>(n), gPlanePaths[n].c_str());
		XPLMDebugString(strbuf);
#endif	
		ptrs.push_back((char *) gPlanePaths[n].c_str());
	}
	ptrs.push_back(NULL);
	
	
	// Attempt to grab multiplayer planes, then analyze.
	int	result = XPLMAcquirePlanes(&(*ptrs.begin()), NULL, NULL);
	if (result) {
		gHasControlOfAIAircraft = true;
		XPLMSetActiveAircraftCount(1);

		int	total, 		active;
		XPLMPluginID	who;

		XPLMCountAircraft(&total, &active, &who);
        
        // Disable AI for the planes...we move them now
        for (int i = 1; i < total; i++)
            XPLMDisableAIForPlane(i);

        // Cleanup multiplayer values...we are in control now
        XPMPInitMultiplayerDataRefs();

		// Register the plane control calls.
		XPLMRegisterDrawCallback(XPMPControlPlaneCount,
			xplm_Phase_Airplanes, 1, /* before*/ 0 /* hide planes*/);

		XPLMRegisterDrawCallback(XPMPControlPlaneCount,
			xplm_Phase_Airplanes, 0, /* after */ (void *) -1 /* show planes*/);
	} else {
		gHasControlOfAIAircraft = false;
		XPLMDebugString("WARNING: " XPMP_CLIENT_LONGNAME " did not acquire multiplayer planes!!\n");
		XPLMDebugString("    Without multiplayer plane control, we cannot fake TCAS or render ACF aircraft!\n");
		XPLMDebugString("    Make sure you remove any plugins that control multiplayer aircraft if you want these features to work\n");
	}

	return "";
}

void XPMPMultiplayerDisable(void)
{
	if (gHasControlOfAIAircraft) {
		XPLMReleasePlanes();
		gHasControlOfAIAircraft = false;
        gPlanePaths.clear();

		XPLMUnregisterDrawCallback(XPMPControlPlaneCount, xplm_Phase_Airplanes, 1, 0);
		XPLMUnregisterDrawCallback(XPMPControlPlaneCount, xplm_Phase_Airplanes, 0, (void *) -1);
	}

}

bool XPMPHasControlOfAIAircraft(void)
{
    return gHasControlOfAIAircraft;
}

const char * 	XPMPLoadCSLPackage(
		const char * inCSLFolder, const char * inRelatedPath, const char * inDoc8643)
{
	bool	problem = false;

	if (!CSL_LoadCSL(inCSLFolder, inRelatedPath, inDoc8643))
		problem = true;

	if (problem)		return "There were problems initializing " XPMP_CLIENT_LONGNAME ".  Please examine X-Plane's error.out file for detailed information.";
	else 				return "";
}

// This routine checks plane loading and grabs anyone we're missing.
void	XPMPLoadPlanesIfNecessary(void)
{
	// if we do not control the MP system, we can not do this.
	if (!gHasControlOfAIAircraft) {
		return;
	}
	int	active, models;
	XPLMPluginID	owner;
	XPLMCountAircraft(&models, &active, &owner);
	if (owner != XPLMGetMyID())
		return;

	if (models > static_cast<int>(gPlanePaths.size()))
		models = static_cast<int>(gPlanePaths.size());
	for (int n = 1; n < models; ++n)
	{
		if (!gPlanePaths[n].empty())
		{
			const char *	ourPath = gPlanePaths[n].c_str();
			char	realPath[512];
			char	fileName[256];
			XPLMGetNthAircraftModel(n, fileName, realPath);
			if (strcmp(ourPath, realPath))
			{
#if DEBUG_MANUAL_LOADING			
				XPLMDebugString("Manually Loading plane: ");
				XPLMDebugString(ourPath);
				XPLMDebugString("\n");
#endif				
				XPLMSetAircraftModel(n, ourPath);
			}
		}
	}

}

int XPMPGetNumberOfInstalledModels(void)
{
	size_t number = 0;
	for (const auto& package : gPackages)
	{
		number += package.planes.size();
	}
	return static_cast<int>(number);
}

void XPMPGetModelInfo(int inIndex, const char** outModelName, const char** outIcao, const char** outAirline, const char** outLivery)
{
	int counter = 0;
	for (const auto& package : gPackages)
	{

		if (counter + static_cast<int>(package.planes.size()) < inIndex + 1)
		{
			counter += static_cast<int>(package.planes.size());
			continue;
		}

		int positionInPackage =  inIndex - counter;
		*outModelName = package.planes[positionInPackage].getModelName().c_str();
		*outIcao = package.planes[positionInPackage].icao.c_str();
		*outAirline = package.planes[positionInPackage].airline.c_str();
		*outLivery = package.planes[positionInPackage].livery.c_str();
		break;
	}
}

/********************************************************************************
 * PLANE OBJECT SUPPORT
 ********************************************************************************/

XPMPPlaneID		XPMPCreatePlane(
		const char *			inICAOCode,
		const char *			inAirline,
		const char *			inLivery,
		XPMPPlaneData_f			inDataFunc,
		void *					inRefcon)
{
	auto plane = std::make_unique<XPMPPlane_t>();
	plane->icao = inICAOCode;
	plane->livery = inLivery;
	plane->airline = inAirline;
	plane->dataFunc = inDataFunc;
	plane->ref = inRefcon;
	plane->model = CSL_MatchPlane(inICAOCode, inAirline, inLivery, &plane->match_quality, true);
	
	plane->pos.size = sizeof(plane->pos);
	plane->surface.size = sizeof(plane->surface);
	plane->radar.size = sizeof(plane->radar);
    plane->infoTexts.size = sizeof(plane->infoTexts);
	plane->posAge = plane->radarAge = plane->surfaceAge = plane->infoAge -1;
	gPlanes.push_back(std::move(plane));
	
	XPMPPlanePtr planePtr = gPlanes.back().get();
	for (XPMPPlaneNotifierVector::iterator iter = gObservers.begin(); iter !=
		 gObservers.end(); ++iter)
	{
		iter->first.first(planePtr, xpmp_PlaneNotification_Created, iter->first.second);
	}
	return planePtr;
}

bool CompareCaseInsensitive(const string &a, const string &b)
{
	return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char aa, char bb) { return toupper(aa) == toupper(bb); });
}

XPMPPlaneID     XPMPCreatePlaneWithModelName(const char *inModelName, const char *inICAOCode, const char *inAirline, const char *inLivery, XPMPPlaneData_f inDataFunc, void *inRefcon)
{
	auto plane = std::make_unique<XPMPPlane_t>();
	plane->icao = inICAOCode;
	plane->livery = inLivery;
	plane->airline = inAirline;
	plane->dataFunc = inDataFunc;
	plane->ref = inRefcon;

	// Find the model
	for (auto &package : gPackages)
	{
		auto cslPlane = std::find_if(package.planes.begin(), package.planes.end(), [inModelName](CSLPlane_t p) { return CompareCaseInsensitive(p.getModelName(), inModelName); });
		if (cslPlane != package.planes.end())
		{
			plane->model = &(*cslPlane);
		}
	}

	if (!plane->model)
	{
		XPLMDebugString("Requested model ");
		XPLMDebugString(inModelName);
		XPLMDebugString(" is unknown! Falling back to own model matching.");
		XPLMDebugString("\n");
		return XPMPCreatePlane(inICAOCode, inAirline, inLivery, inDataFunc, inRefcon);
	}

	plane->pos.size = sizeof(plane->pos);
	plane->surface.size = sizeof(plane->surface);
	plane->radar.size = sizeof(plane->radar);
    plane->infoTexts.size = sizeof(plane->infoTexts);
	plane->posAge = plane->radarAge = plane->surfaceAge = plane->infoAge = -1;
	gPlanes.push_back(std::move(plane));

	XPMPPlanePtr planePtr = gPlanes.back().get();
	for (XPMPPlaneNotifierVector::iterator iter = gObservers.begin(); iter !=
		 gObservers.end(); ++iter)
	{
		iter->first.first(planePtr, xpmp_PlaneNotification_Created, iter->first.second);
	}
	return planePtr;
}

void			XPMPDestroyPlane(XPMPPlaneID inID)
{
	XPMPPlaneVector::iterator iter;
	XPMPPlanePtr plane = XPMPPlaneFromID(inID, &iter);

	for (XPMPPlaneNotifierVector::iterator iter2 = gObservers.begin(); iter2 !=
		 gObservers.end(); ++iter2)
	{
		iter2->first.first(plane, xpmp_PlaneNotification_Destroyed, iter2->first.second);
	}
	gPlanes.erase(iter);
}

int	XPMPChangePlaneModel(
		XPMPPlaneID				inPlaneID,
		const char *			inICAOCode,
		const char *			inAirline,
		const char *			inLivery)
{
	XPMPPlanePtr plane = XPMPPlaneFromID(inPlaneID);
	plane->icao = inICAOCode;
	plane->airline = inAirline;
	plane->livery = inLivery;
	plane->model = CSL_MatchPlane(inICAOCode, inAirline, inLivery, &plane->match_quality, true);

	// we're changing model, we must flush the resource handles so they get reloaded.
	plane->objHandle = NULL;
	plane->texHandle = NULL;
	plane->texLitHandle = NULL;
	plane->objState = {};
	plane->texState = {};
	plane->texLitState = {};

	for (XPMPPlaneNotifierVector::iterator iter2 = gObservers.begin(); iter2 !=
		 gObservers.end(); ++iter2)
	{
		iter2->first.first(plane, xpmp_PlaneNotification_ModelChanged, iter2->first.second);
	}

	return plane->match_quality;
}

/*
 * XPMPGetPlaneModel
 *
 * Returns a textual description of the model in use
 * Returns required buf size, i.e. length of description.
 * Negative values indicate failure (wrong PlaneID).
 *
 */
int     XPMPGetPlaneModelName(
            XPMPPlaneID         inPlaneID,
            char *              outTxtBuf,
            int                 outTxtBufSize)
{
    std::string modelName ("(none)");
    
    // fetch plane pointer, exit if NULL
    XPMPPlanePtr plane = XPMPPlaneFromID(inPlaneID);
    if (!plane) return -1;
    
    // get model name if there is a current model, otherwise "(none)"
    if ( plane->model )
    {
        modelName = plane->model->getModelName();
    
        // if more or less empty use type, airline, file_path
        if (modelName == " ")
            modelName =
                plane->model->icao + " " +
                plane->model->airline + " " +
                plane->model->file_path;
    }
    
    // copy into outBuffer as much as possible
    if (outTxtBuf && outTxtBufSize > 1)
        strncpy(outTxtBuf, modelName.c_str(), outTxtBufSize);
    
    // return required string len
    return (int)modelName.length();
}
/*** END OF INSERT ***/

void	XPMPSetDefaultPlaneICAO(
		const char *			inICAO)
{
	gDefaultPlane = inICAO;
}						

long			XPMPCountPlanes(void)
{
	return static_cast<long>(gPlanes.size());
}

XPMPPlaneID		XPMPGetNthPlane(
		long 					index)
{
	if ((index < 0) || (index >= static_cast<long>(gPlanes.size())))
		return NULL;

	return gPlanes[index].get();
}							


void XPMPGetPlaneICAOAndLivery(
		XPMPPlaneID				inPlane,
		char *					outICAOCode,	// Can be NULL
		char *					outLivery)
{
	XPMPPlanePtr	plane = XPMPPlaneFromID(inPlane);

	if (outICAOCode)
		strcpy(outICAOCode,plane->icao.c_str());
	if (outLivery)
		strcpy(outLivery,plane->livery.c_str());
}	

void			XPMPRegisterPlaneNotifierFunc(
		XPMPPlaneNotifier_f		inFunc,
		void *					inRefcon)
{
	gObservers.push_back(XPMPPlaneNotifierTripple(XPMPPlaneNotifierPair(inFunc, inRefcon), XPLMGetMyID()));
}					

void			XPMPUnregisterPlaneNotifierFunc(
		XPMPPlaneNotifier_f		inFunc,
		void *					inRefcon)
{
	XPMPPlaneNotifierVector::iterator iter = std::find(
				gObservers.begin(), gObservers.end(), XPMPPlaneNotifierTripple(XPMPPlaneNotifierPair(inFunc, inRefcon), XPLMGetMyID()));
	if (iter != gObservers.end())
		gObservers.erase(iter);
}					

XPMPPlaneCallbackResult			XPMPGetPlaneData(
		XPMPPlaneID					inPlane,
		XPMPPlaneDataType			inDataType,
		void *						outData)
{
	XPMPPlanePtr	plane = XPMPPlaneFromID(inPlane);
	
	XPMPPlaneCallbackResult result = xpmpData_Unavailable;
	
	int now = XPLMGetCycleNumber();

	switch(inDataType) {
	case xpmpDataType_Position:
	{
		if (plane->posAge != now)
		{
            // HACK to reduce jitter in external camera applications:
            // We draw the plane at the _previous_ frame's position,
            // because this is the position known to the camera app;
            // the camera app's callback to retrieve camera position is already
            // called by this time and the camera position might base on
            // the multiplayer dataRefs...of the _previous_ frame.
            // From the application we pull the _next_ frame,
            // which is the pos to be reported in the multiplayer vars
            plane->pos = plane->nextPos;
            result = plane->dataFunc(plane, inDataType, &plane->nextPos, plane->ref);
			if (result == xpmpData_NewData)
				plane->posAge = now;
		}

		XPMPPlanePosition_t *	posD = (XPMPPlanePosition_t *) outData;
		memcpy(posD, &plane->pos, XPMP_TMIN(posD->size, plane->pos.size));
		result = xpmpData_Unchanged;

		break;
	}
	case xpmpDataType_Surfaces:
	{
		if (plane->surfaceAge != now)
		{
			result = plane->dataFunc(plane, inDataType, &plane->surface, plane->ref);
			if (result == xpmpData_NewData)
				plane->surfaceAge = now;
		}

		XPMPPlaneSurfaces_t *	surfD = (XPMPPlaneSurfaces_t *) outData;
		memcpy(surfD, &plane->surface, XPMP_TMIN(surfD->size, plane->surface.size));
		result = xpmpData_Unchanged;

		break;
	}
	case xpmpDataType_Radar:
	{
		if (plane->radarAge != now)
		{
			result = plane->dataFunc(plane, inDataType, &plane->radar, plane->ref);
			if (result == xpmpData_NewData)
				plane->radarAge = now;
		}

		XPMPPlaneRadar_t *	radD = (XPMPPlaneRadar_t *) outData;
		memcpy(radD, &plane->radar, XPMP_TMIN(radD->size, plane->radar.size));
		result = xpmpData_Unchanged;

		break;
	}
    case xpmpDataType_InfoTexts:
    {
        if (plane->infoAge != now)
        {
            result = plane->dataFunc(plane, inDataType, &plane->infoTexts, plane->ref);
            if (result == xpmpData_NewData)
                plane->infoAge = now;
        }
        // Don't know why `result` is _always_ overwritten in legacy code above...
        // we do process the difference between xpmpData_NewData and xpmpData_Unchanged
        // as returned by the called app, so we do _not_ override the result code.
        // So we set xpmpData_Unchanged only here in the else branch if we don't
        // even call the callback:
        else
            result = xpmpData_Unchanged;

        XPMPInfoTexts_t *    infoD = (XPMPInfoTexts_t *) outData;
        memcpy(infoD, &plane->infoTexts, XPMP_TMIN(infoD->size, plane->infoTexts.size));
        
        break;
    }
	}
	return result;
}

XPMPPlanePtr	XPMPPlaneFromID(XPMPPlaneID inID, XPMPPlaneVector::iterator * outIter)
{
	assert(inID);
	if (outIter)
	{
		*outIter = std::find_if(gPlanes.begin(), gPlanes.end(), [inID] (const auto &p)
		{
			return p.get() == inID;
		});
		assert(*outIter != gPlanes.end());
	}
	return static_cast<XPMPPlanePtr>(inID);
}

void		XPMPSetPlaneRenderer(
		XPMPRenderPlanes_f  		inRenderer,
		void * 						inRef)
{
	gRenderer = inRenderer;
	gRendererRef = inRef;
}					

/********************************************************************************
 * RENDERING
 ********************************************************************************/

// This callback ping-pongs the multiplayer count up and back depending 
// on whether we're drawing airplanes or not.
// (before airplane drawing: reset to 1, so that no AI models are drawn by X-Plane,
//  after airplane drawing: back up to the actual number of TCAS aircrafts,
//                          so that everbody believes these many AI aircrafts are out there.)
int	XPMPControlPlaneCount(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               inRefcon)
{
	if (!gHasControlOfAIAircraft) {
		return 1;
	}
	if (inRefcon == NULL)
	{
		XPLMSetActiveAircraftCount(1);
	} else {
		XPLMSetActiveAircraftCount(gEnableCount);
	}
	return 1;
}


// This routine draws the actual planes.
int	XPMPRenderMultiplayerPlanes(
		XPLMDrawingPhase     /*inPhase*/,
		int                  /*inIsBefore*/,
		void *               /*inRefcon*/)
{
	static int is_blend = 0;
	
	static XPLMDataRef wrt = XPLMFindDataRef("sim/graphics/view/world_render_type");
	static XPLMDataRef prt = XPLMFindDataRef("sim/graphics/view/plane_render_type");
	
	int is_shadow = wrt != NULL && XPLMGetDatai(wrt) != 0;
	
	if(prt)
		is_blend = XPLMGetDatai(prt) == 2;

	if (gRenderer)
		gRenderer(is_shadow ? 0 : is_blend,gRendererRef);
	else
		XPMPDefaultPlaneRenderer(is_shadow ? 0 : is_blend);
	if(!is_shadow)
		is_blend = 1 - is_blend;
	return 1;
}

bool			XPMPIsICAOValid(
		const char *				inICAO)
{
	return CSL_MatchPlane(inICAO, "", "", NULL, false) != NULL;
}

int 		XPMPGetPlaneModelQuality(
		XPMPPlaneID 				inPlane)
{
	XPMPPlanePtr thisPlane = XPMPPlaneFromID(inPlane);

	return thisPlane->match_quality;
}

int			XPMPModelMatchQuality(
		const char *				inICAO,
		const char *				inAirline,
		const char *				inLivery)
{
	int 	matchQuality = -1;
	CSL_MatchPlane(inICAO, inAirline, inLivery, &matchQuality, false);

	return matchQuality;
}

void		XPMPDumpOneCycle(void)
{
	CSL_Dump();
	gDumpOneRenderCycle = true;
}
