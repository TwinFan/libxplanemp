/*
 * Copyright (c) 2005, Ben Supnik and Chris Serio.
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

#include "XPMPMultiplayerObj.h"
#include "XPMPMultiplayerVars.h"

//#include "PlatformUtils.h"
#include "XObjReadWrite.h"
#include "TexUtils.h"
#include "XUtils.h"

#include <map>
#include <vector>
#include <cmath>
#include <cstdio>
#include <queue>
#include <fstream>

#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"

#define DEBUG_NORMALS 0
#define	DISABLE_SHARING 0
#define BLEND_NORMALS 1

#ifdef IBM
#define snprintf _snprintf
#endif

// Set this to 1 to get resource loading and unloading diagnostics
#define DEBUG_RESOURCE_CACHE 0

int xpmp_spare_texhandle_decay_frames = -1;

using namespace std;

const	double	kMetersToNM = 0.000539956803;
// Some color constants
//const	float	kNavLightRed[] = {1.0, 0.0, 0.2, 0.6};
//const	float	kNavLightGreen[] = {0.0, 1.0, 0.3, 0.6};
//const	float	kLandingLight[]	= {1.0, 1.0, 0.7, 0.6};
//const	float	kStrobeLight[]	= {1.0, 1.0, 1.0, 0.4};
const	float	kNavLightRed[] = {1.0f, 0.0f, 0.2f, 0.5f};
const	float	kNavLightGreen[] = {0.0f, 1.0f, 0.3f, 0.5f};
const	float	kLandingLight[]	= {1.0f, 1.0f, 0.7f, 0.6f};
const	float	kStrobeLight[]	= {1.0f, 1.0f, 1.0f, 0.7f};
const	float	kTaxiLight[] = {1.0f, 1.0f, 0.7f, 0.6f};

static	int sLightTexture = -1;

static void MakePartialPathNativeObj(string& io_str)
{
	//	char sep = *XPLMGetDirectorySeparator();
	for(size_t i = 0; i < io_str.size(); ++i)
		if(io_str[i] == '/' || io_str[i] == ':' || io_str[i] == '\\')
			io_str[i] = '/';
}

static	XPLMDataRef sFOVRef = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
static	float		sFOV = 60.0;

bool 	NormalizeVec(float vec[3])
{
	float	len=sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
	if (len>0.0)
	{
		len = 1.0f / len;
		vec[0] *= len;
		vec[1] *= len;
		vec[2] *= len;
		return true;
	}
	return false;
}

void	CrossVec(float a[3], float b[3], float dst[3])
{
	dst[0] = a[1] * b[2] - a[2] * b[1] ;
	dst[1] = a[2] * b[0] - a[0] * b[2] ;
	dst[2] = a[0] * b[1] - a[1] * b[0] ;
}

// Adds a point to our pool and returns it's index.
// If one already exists in the same location, we
// just return that index
int	OBJ_PointPool::AddPoint(float xyz[3], float st[2])
{
#if !DISABLE_SHARING
	// Use x as the key...see if we can find it
	for(size_t n = 0; n < mPointPool.size(); n += 8)
	{
		if((xyz[0] == mPointPool[n]) &&
				(xyz[1] == mPointPool[n+1]) &&
				(xyz[2] == mPointPool[n+2]) &&
				(st[0] == mPointPool[n+3]) &&
				(st[1] == mPointPool[n+4]))
			return static_cast<int>(n/8);	// Clients care about point # not array index
	}
#endif	

	// If we're here, no match was found so we add it to the pool
	// Add XYZ
	mPointPool.push_back(xyz[0]);	mPointPool.push_back(xyz[1]);	mPointPool.push_back(xyz[2]);
	// Add ST
	mPointPool.push_back(st[0]); mPointPool.push_back(st[1]);
	// Allocate some space for the normal later
	mPointPool.push_back(0.0); mPointPool.push_back(0.0); mPointPool.push_back(0.0);
	return (static_cast<int>(mPointPool.size())/8)-1;
}

// This function sets up OpenGL for our point pool
void OBJ_PointPool::PreparePoolToDraw()
{
	// Setup our triangle data (20 represents 5 elements of 4 bytes each
	// namely s,t,xn,yn,zn)
	glVertexPointer(3, GL_FLOAT, 32, &(*mPointPool.begin()));
	// Set our texture data (24 represents 6 elements of 4 bytes each
	// namely xn, yn, zn, x, y, z. We start 3 from the beginning to skip
	// over x, y, z initially.
	glClientActiveTextureARB(GL_TEXTURE1);
	glTexCoordPointer(2, GL_FLOAT, 32, &(*(mPointPool.begin() + 3)));
	glClientActiveTextureARB(GL_TEXTURE0);
	glTexCoordPointer(2, GL_FLOAT, 32, &(*(mPointPool.begin() + 3)));
	// Set our normal data...
	glNormalPointer(GL_FLOAT, 32, &(*(mPointPool.begin() + 5)));
}

void OBJ_PointPool::CalcTriNormal(int idx1, int idx2, int idx3)
{
	if (mPointPool[idx1*8  ]==mPointPool[idx2*8  ]&&
			mPointPool[idx1*8+1]==mPointPool[idx2*8+1]&&
			mPointPool[idx1*8+2]==mPointPool[idx2*8+2])		return;
	if (mPointPool[idx1*8  ]==mPointPool[idx3*8  ]&&
			mPointPool[idx1*8+1]==mPointPool[idx3*8+1]&&
			mPointPool[idx1*8+2]==mPointPool[idx3*8+2])		return;
	if (mPointPool[idx2*8  ]==mPointPool[idx3*8  ]&&
			mPointPool[idx2*8+1]==mPointPool[idx3*8+1]&&
			mPointPool[idx2*8+2]==mPointPool[idx3*8+2])		return;

	// idx2->idx1 cross idx1->idx3 = normal product
	float	v1[3], v2[3], n[3];
	v1[0] = mPointPool[idx2*8  ] - mPointPool[idx1*8  ];
	v1[1] = mPointPool[idx2*8+1] - mPointPool[idx1*8+1];
	v1[2] = mPointPool[idx2*8+2] - mPointPool[idx1*8+2];

	v2[0] = mPointPool[idx2*8  ] - mPointPool[idx3*8  ];
	v2[1] = mPointPool[idx2*8+1] - mPointPool[idx3*8+1];
	v2[2] = mPointPool[idx2*8+2] - mPointPool[idx3*8+2];
	
	// We do NOT normalize the cross product; we want larger triangles
	// to make bigger normals.  When we blend them, bigger sides will
	// contribute more to the normals.  We'll normalize the normals
	// after the blend is done.
	CrossVec(v1, v2, n);
	mPointPool[idx1*8+5] += n[0];
	mPointPool[idx1*8+6] += n[1];
	mPointPool[idx1*8+7] += n[2];

	mPointPool[idx2*8+5] += n[0];
	mPointPool[idx2*8+6] += n[1];
	mPointPool[idx2*8+7] += n[2];

	mPointPool[idx3*8+5] += n[0];
	mPointPool[idx3*8+6] += n[1];
	mPointPool[idx3*8+7] += n[2];
}

inline void swapped_add(float& a, float& b)
{
	float a_c = a;
	float b_c = b;
	a += b_c;
	b += a_c;
}

void OBJ_PointPool::NormalizeNormals(void)
{
	// Average all normals of same-point, different texture points?  Why is this needed?
	// Well...the problem is that we get non-blended normals around the 'seam' where the ACF fuselage
	// is put together...at this point the separate top and bottom texes touch.  Their color will
	// be the same but the S&T coords won't.  If we have slightly different normals and the sun is making
	// shiney specular hilites, the discontinuity is real noticiable.
#if BLEND_NORMALS
	for (size_t n = 0; n < mPointPool.size(); n += 8)
	{
		for (size_t m = 0; m < mPointPool.size(); m += 8)
			if (mPointPool[n  ]==mPointPool[m  ] &&
					mPointPool[n+1]==mPointPool[m+1] &&
					mPointPool[n+2]==mPointPool[m+2] &&
					m != n)
			{
				swapped_add(mPointPool[n+5], mPointPool[m+5]);
				swapped_add(mPointPool[n+6], mPointPool[m+6]);
				swapped_add(mPointPool[n+7], mPointPool[m+7]);
			}
	}
#endif	
	for (size_t n = 5; n < mPointPool.size(); n += 8)
	{
		NormalizeVec(&mPointPool[n]);
	}
}

// This is a debug routine that will draw each vertex's normals
void OBJ_PointPool::DebugDrawNormals()
{
	XPLMSetGraphicsState(0, 0, 0, 0, 0, 1, 0);
	glColor3f(1.0, 0.0, 1.0);
	glBegin(GL_LINES);
	for(size_t n = 0; n < mPointPool.size(); n+=8)
	{
		glVertex3f(mPointPool[n], mPointPool[n+1], mPointPool[n+2]);
		glVertex3f(mPointPool[n] + mPointPool[n+5], mPointPool[n+1] + mPointPool[n+1+5],
				mPointPool[n+2] + mPointPool[n+2+5]);
	}
	glEnd();
}

static	map<string, int>	sTexes;
static vector<ObjInfo_t>	sObjects;

static ObjManager gObjManager(OBJ_LoadModelAsync);
static TextureManager gTextureManager(OBJ_LoadTexture);

static std::queue<GLuint> sFreedTextures;

/*****************************************************
		Utility functions to handle OBJ stuff
******************************************************/

int OBJ_LoadLightTexture(const string &inFilePath, bool inForceMaxTex)
{
	string	path(inFilePath);
	if (sTexes.count(path) > 0)
		return sTexes[path];

	int derez = 5 - gIntPrefsFunc("planes", "resolution", 5);
	if (inForceMaxTex)
		derez = 0;

	GLuint texNum = 0;
	bool ok = LoadTextureFromFile(path, true, false, true, derez, &texNum, NULL, NULL);
	if (!ok) return 0;

	sTexes[path] = texNum;
	return texNum;
}

void DeleteTexture(CSLTexture_t* texture)
{
#if DEBUG_RESOURCE_CACHE
	XPLMDebugString(XPMP_CLIENT_NAME ": Released texture id=");
	char buf[32];
	sprintf(buf,"%d", texture->id);
	XPLMDebugString(buf);
	XPLMDebugString(" (");
	XPLMDebugString(texture->path.c_str());
	XPLMDebugString(")\n");
#endif

	if (sFreedTextures.size() >= MAX_SPARE_TEXHANDLES) {
		GLuint textures[] = { texture->id };
		glDeleteTextures(1, textures);
	} else {
		sFreedTextures.push(texture->id);
	}
	delete texture;
}

TextureManager::Future OBJ_LoadTexture(const string &path)
{
	return std::async(std::launch::async, [path]
	{
#if DEBUG_RESOURCE_CACHE
		XPLMDebugString(XPMP_CLIENT_NAME ": Loading texture ");
		XPLMDebugString("(");
		XPLMDebugString(path.c_str());
		XPLMDebugString(")\n");
#endif

		int derez = 5 - gIntPrefsFunc("planes", "resolution", 5);
		ImageInfo im;
		CSLTexture_t texture;
		texture.id = 0;
		if (!LoadImageFromFile(path, true, derez, im, NULL, NULL))
		{
			XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: ");
			XPLMDebugString(path.c_str());
			XPLMDebugString(" failed to load.");
			XPLMDebugString("\n");
			texture.loadStatus = Failed;
			return std::make_shared<CSLTexture_t>(texture);
		}
		if (!VerifyTextureImage(path, im)) {
			// VTI reports it's own errors.
			texture.loadStatus = Failed;
			return std::make_shared<CSLTexture_t>(texture);			
		}
		texture.path = path;
		texture.im = im;
		texture.loadStatus = Succeeded;
		return TextureManager::ResourceHandle(new CSLTexture_t(texture), DeleteTexture);
	});
}

bool	OBJ_Init(const char * inTexturePath)
{
	// Now that we've successfully loaded an aircraft,
	// we can load our lighting texture
	static bool firstTime = true;
	if(firstTime)
	{
		sLightTexture = OBJ_LoadLightTexture(inTexturePath, true);
		firstTime = false;
	}
	return sLightTexture != 0;
}

void DeleteObjInfo(ObjInfo_t* objInfo)
{
#if DEBUG_RESOURCE_CACHE
	XPLMDebugString(XPMP_CLIENT_NAME ": Released OBJ ");
	XPLMDebugString("(");
	XPLMDebugString(objInfo->path.c_str());
	XPLMDebugString(")\n");
#endif

	delete objInfo;
}

// Load one model - returns nullptr handle if it can't be loaded.
ObjManager::ResourceHandle OBJ_LoadModel(const string &inFilePath)
{
#if DEBUG_RESOURCE_CACHE
		XPLMDebugString(XPMP_CLIENT_NAME ": Loading OBJ ");
		XPLMDebugString("(");
		XPLMDebugString(inFilePath.c_str());
		XPLMDebugString(")\n");
#endif

	ObjInfo_t objInfo;
	string path(inFilePath);

	bool ok = XObjReadWrite::read(path, objInfo.obj);
	if (!ok)
	{
		XPLMDebugString(XPMP_CLIENT_NAME ": WARNING: ");
		XPLMDebugString(path.c_str());
		XPLMDebugString(" failed to load.");
		XPLMDebugString("\n");
		objInfo.loadStatus = Failed;
		return ObjManager::ResourceHandle(new ObjInfo_t(objInfo), DeleteObjInfo);
	}

	MakePartialPathNativeObj(objInfo.obj.texture);
	objInfo.path = path;
	string tex_path(path);
	string::size_type p = tex_path.find_last_of("\\:/");//XPLMGetDirectorySeparator());
	tex_path.erase(p+1);
	tex_path += objInfo.obj.texture;
	tex_path += ".png";

	objInfo.defaultTexture = tex_path;
	// fixme: needed?
	objInfo.texnum = -1;
	objInfo.texnum_lit = -1;
	objInfo.defaultLitTexture = OBJ_GetLitTextureByTexture(objInfo.defaultTexture);

	// We prescan all of the commands to see if there's ANY LOD. If there's
	// not then we need to add one ourselves. If there is, we will find it
	// naturally later.
	bool foundLOD = false;
	for (const auto &cmd : objInfo.obj.cmds)
	{
		if((cmd.cmdType == type_Attr) && (cmd.cmdID == attr_LOD))
			foundLOD = true;
	}
	if(foundLOD == false)
	{
		objInfo.lods.push_back(LODObjInfo_t());
		objInfo.lods.back().nearDist = 0;
		objInfo.lods.back().farDist = 40000;
	}

	// Go through all of the commands for this object and filter out the polys
	// and the lights.
	for (const auto &cmd : objInfo.obj.cmds)
	{
		switch(cmd.cmdType) {
		case type_Attr:
			if(cmd.cmdID == attr_LOD)
			{
				// We've found a new LOD section so save this
				// information in a new struct. From now on and until
				// we hit this again, all data is for THIS LOD instance.
				objInfo.lods.push_back(LODObjInfo_t());
				// Save our visible LOD range
				objInfo.lods.back().nearDist = cmd.attributes[0];
				objInfo.lods.back().farDist = cmd.attributes[1];
			}
			break;
		case type_PtLine:
			if(cmd.cmdID == obj_Light)
			{
				// For each light we've found, copy the data into our
				// own light vector
				for(size_t n = 0; n < cmd.rgb.size(); n++)
				{
					objInfo.lods.back().lights.push_back(LightInfo_t());
					objInfo.lods.back().lights.back().xyz[0] = cmd.rgb[n].v[0];
					objInfo.lods.back().lights.back().xyz[1] = cmd.rgb[n].v[1];
					objInfo.lods.back().lights.back().xyz[2] = cmd.rgb[n].v[2];
					objInfo.lods.back().lights.back().rgb[0] = static_cast<int>(cmd.rgb[n].rgb[0]);
					objInfo.lods.back().lights.back().rgb[1] = static_cast<int>(cmd.rgb[n].rgb[1]);
					objInfo.lods.back().lights.back().rgb[2] = static_cast<int>(cmd.rgb[n].rgb[2]);
				}
			}
			break;
		case type_Poly:
		{
			vector<int> indexes;
			// First get our point pool setup with all verticies
			for(size_t n = 0; n < cmd.st.size(); n++)
			{
				float xyz[3], st[2];
				int index;

				xyz[0] = cmd.st[n].v[0];
				xyz[1] = cmd.st[n].v[1];
				xyz[2] = cmd.st[n].v[2];
				st[0]  = cmd.st[n].st[0];
				st[1]  = cmd.st[n].st[1];
				index = objInfo.lods.back().pointPool.AddPoint(xyz, st);
				indexes.push_back(index);
			}

			switch(cmd.cmdID) {
			case obj_Tri:
				for(size_t n = 0; n < indexes.size(); ++n)
				{
					objInfo.lods.back().triangleList.push_back(indexes[n]);
				}
				break;
			case obj_Tri_Fan:
				for(size_t n = 2; n < indexes.size(); n++)
				{
					objInfo.lods.back().triangleList.push_back(indexes[0  ]);
					objInfo.lods.back().triangleList.push_back(indexes[n-1]);
					objInfo.lods.back().triangleList.push_back(indexes[n  ]);
				}
				break;
			case obj_Tri_Strip:
			case obj_Quad_Strip:
				for(size_t n = 2; n < indexes.size(); n++)
				{
					if((n % 2) == 1)
					{
						objInfo.lods.back().triangleList.push_back(indexes[n - 2]);
						objInfo.lods.back().triangleList.push_back(indexes[n]);
						objInfo.lods.back().triangleList.push_back(indexes[n - 1]);
					}
					else
					{
						objInfo.lods.back().triangleList.push_back(indexes[n - 2]);
						objInfo.lods.back().triangleList.push_back(indexes[n - 1]);
						objInfo.lods.back().triangleList.push_back(indexes[n]);
					}
				}
				break;
			case obj_Quad:
				for(size_t n = 3; n < indexes.size(); n += 4)
				{
					objInfo.lods.back().triangleList.push_back(indexes[n-3]);
					objInfo.lods.back().triangleList.push_back(indexes[n-2]);
					objInfo.lods.back().triangleList.push_back(indexes[n-1]);
					objInfo.lods.back().triangleList.push_back(indexes[n-3]);
					objInfo.lods.back().triangleList.push_back(indexes[n-1]);
					objInfo.lods.back().triangleList.push_back(indexes[n  ]);
				}
				break;
			}
		}
			break;
		}
	}

	// Calculate our normals for all LOD's
	for (size_t i = 0; i < objInfo.lods.size(); i++)
	{
		for (size_t n = 0; n < objInfo.lods[i].triangleList.size(); n += 3)
		{
			objInfo.lods[i].pointPool.CalcTriNormal(
						objInfo.lods[i].triangleList[n],
						objInfo.lods[i].triangleList[n+1],
					objInfo.lods[i].triangleList[n+2]);
		}
		objInfo.lods[i].pointPool.NormalizeNormals();
		objInfo.lods[i].dl = 0;
	}
	objInfo.obj.cmds.clear();
	objInfo.loadStatus = Succeeded;
	return ObjManager::ResourceHandle(new ObjInfo_t(objInfo), DeleteObjInfo);
}

ObjManager::Future OBJ_LoadModelAsync(const string &inFilePath)
{
	return std::async(std::launch::async, [inFilePath]
	{
		return OBJ_LoadModel(inFilePath);
	});
}

std::string OBJ_DefaultModel(const string &path)
{
	XObj xobj;
	int version;
	XObjReadWrite::readHeader(path, version, xobj);
	return xobj.texture;
}

/*****************************************************
			Aircraft Model Drawing
******************************************************/

/*
 * xpmpGetGlTextureId tries to return a spare texture handle from our pool of
 * unused GL texture handles.
 *
 * if we're out, return 0.  LoadTextureFromMemory knows that a textureid of 0 means
 * that it has to get a new handle from the GL layer anyway.
 *
 * If we have any textures left after this operation, we bump the timer for texture handle release.
 */
GLuint xpmpGetGlTextureId() {
	GLuint	retHandle = 0;
	if (! sFreedTextures.empty()) {
		retHandle = sFreedTextures.front();
		sFreedTextures.pop();
	}
	if (sFreedTextures.empty()) {
		xpmp_spare_texhandle_decay_frames = -1;
	} else {
		xpmp_spare_texhandle_decay_frames = SPARE_TEXHANDLES_DECAY_FRAMES;
	}
	return retHandle;
}

/* OBJ_MaintainTextures should be called every frame we render.  It cleans up the spare texture
 * pool so it doesn't eat texture memory unnecessarily.
 */
void OBJ_MaintainTextures() {
	if (xpmp_spare_texhandle_decay_frames >= 0) {
		if (--xpmp_spare_texhandle_decay_frames <= 0) {
			GLuint textures[] = { xpmpGetGlTextureId() };
			if (textures[0] != 0) {
				glDeleteTextures(1, textures);
			}
			// we don't need to reset the timer ourselves, using xpmpGetGlTextureId has done that for us.
		}
	}
}

// Note that texID and litTexID are OPTIONAL! They will only be filled
// in if the user wants to override the default texture specified by the
// obj file
void	OBJ_PlotModel(XPMPPlane_t *plane, float inDistance, double /*inX*/,
					  double /*inY*/, double /*inZ*/, double /*inPitch*/, double /*inRoll*/, double /*inHeading*/)
{
	if (! plane->objHandle)
	{
		plane->objHandle = gObjManager.get(plane->model->file_path, &plane->objState);
		if (plane->objHandle && plane->objHandle->loadStatus == Failed)
		{
			// Failed to load
			XPLMDebugString("Skipping ");
			XPLMDebugString(plane->model->getModelName().c_str());
			XPLMDebugString(" since object could not be loaded.");
			XPLMDebugString("\n");
		}
	}
	if (! plane->objHandle || plane->objHandle->loadStatus == Failed) { return; }

	// Try to load a texture if not yet done. If one can't be loaded continue without texture
	if (! plane->texHandle)
	{
		string texturePath = plane->model->texturePath;
		if (texturePath.empty()) { texturePath = plane->objHandle->defaultTexture; }
		plane->texHandle = gTextureManager.get(texturePath, &plane->texState);

		// Async loading completed with failure
		if (plane->texHandle && plane->texHandle->loadStatus == Failed)
		{
			// Failed to load
			XPLMDebugString("Texture for ");
			XPLMDebugString(plane->model->getModelName().c_str());
			XPLMDebugString(" cannot be loaded.");
			XPLMDebugString("\n");
		}
	}

	auto model = plane->model;
	// Try to load a texture if not yet done. If one can't be loaded continue without texture
	if (! plane->texLitHandle)
	{
		string texturePath = model->textureLitPath;
		if (texturePath.empty()) { texturePath = plane->objHandle->defaultLitTexture; }
		plane->texLitHandle = gTextureManager.get(texturePath, &plane->texLitState);
	}

	if (plane->texHandle && plane->texHandle->loadStatus == Succeeded && !plane->texHandle->id)
	{
		plane->texHandle->id = xpmpGetGlTextureId();
		LoadTextureFromMemory(plane->texHandle->im, true, false, true, plane->texHandle->id);

#if DEBUG_RESOURCE_CACHE
		XPLMDebugString(XPMP_CLIENT_NAME ": Finished loading of texture id=");
		char buf[32];
		sprintf(buf,"%d", plane->texHandle->id);
		XPLMDebugString(buf);
		XPLMDebugString("\n");
#endif
	}

	if (plane->texLitHandle && plane->texLitHandle->loadStatus == Succeeded && !plane->texLitHandle->id)
	{
		plane->texLitHandle->id = xpmpGetGlTextureId();
		LoadTextureFromMemory(plane->texLitHandle->im, true, false, true, plane->texLitHandle->id);

#if DEBUG_RESOURCE_CACHE
		XPLMDebugString(XPMP_CLIENT_NAME ": Finished loading of texture id=");
		char buf[32];
		sprintf(buf,"%d", plane->texLitHandle->id);
		XPLMDebugString(buf);
		XPLMDebugString("\n");
#endif
	}

	auto obj = plane->objHandle.get();
	// Find out what LOD we need to draw
	int lodIdx = -1;
	for(size_t n = 0; n < obj->lods.size(); n++)
	{
		if((inDistance >= obj->lods[n].nearDist) &&
				(inDistance <= obj->lods[n].farDist))
		{
			lodIdx = static_cast<int>(n);
			break;
		}
	}
	// If we didn't find a good LOD bin, we don't draw!
	if(lodIdx == -1)
		return;

	// pointPool is and always was empty! returning early
	if(obj->lods[lodIdx].pointPool.Size()==0 && obj->lods[lodIdx].dl == 0)
		return;

	static XPLMDataRef	night_lighting_ref = XPLMFindDataRef("sim/graphics/scenery/percent_lights_on");
	bool	use_night = XPLMGetDataf(night_lighting_ref) > 0.25;

	int tex = 0;
	int lit = 0;
	auto texture = plane->texHandle.get();
	if(texture && texture->id)
	{
		tex = texture->id;
	}

	auto litTexure = plane->texLitHandle.get();
	if (litTexure && litTexure->id)
	{
		lit = litTexure->id;
	}

	if (!use_night)	lit = 0;
	if (tex == 0) lit = 0;
	XPLMSetGraphicsState(1, (tex != 0) + (lit != 0), 1, 1, 1, 1, 1);
	if (tex != 0)	XPLMBindTexture2d(tex, 0);
	if (lit != 0)	XPLMBindTexture2d(lit, 1);
	
	if (tex) { glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); }
	if (lit) { glActiveTextureARB(GL_TEXTURE1); glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_ADD); glActiveTextureARB(GL_TEXTURE0); }
	
	if (obj->lods[lodIdx].dl == 0)
	{
		obj->lods[lodIdx].dl = glGenLists(1);
		
		GLint xpBuffer;
		// See if the card even has VBO. If it does, save xplane's pointer
		// and bind to 0 for us.
#if IBM
		if(glBindBufferARB)
#endif
		{
			glGetIntegerv(GL_ARRAY_BUFFER_BINDING_ARB, &xpBuffer);
			glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
		}
		// Save XPlanes OpenGL state
		glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
		// Setup OpenGL pointers to our pool
		obj->lods[lodIdx].pointPool.PreparePoolToDraw();
		// Enable vertex data sucking
		glEnableClientState(GL_VERTEX_ARRAY);
		// Enable normal array sucking
		glEnableClientState(GL_NORMAL_ARRAY);
		// Enable texture coordinate data sucking
		glClientActiveTextureARB(GL_TEXTURE1);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glClientActiveTextureARB(GL_TEXTURE0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		// Disable colors - maybe x-plane left it around.
		glDisableClientState(GL_COLOR_ARRAY);

		glNewList(obj->lods[lodIdx].dl, GL_COMPILE);
		// Kick OpenGL and draw baby!
		glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(obj->lods[lodIdx].triangleList.size()),
					   GL_UNSIGNED_INT, &(*obj->lods[lodIdx].triangleList.begin()));

#if DEBUG_NORMALS
		obj->lods[lodIdx].pointPool.DebugDrawNormals();
		XPLMSetGraphicsState(1, (tex != 0) + (lit != 0), 1, 1, 1, 1, 1);
#endif

		glEndList();

		// Disable vertex data sucking
		glDisableClientState(GL_VERTEX_ARRAY);
		// Disable texture coordinate data sucking
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		// Disable normal array sucking
		glDisableClientState(GL_NORMAL_ARRAY);

		// Restore Xplane's OpenGL State
		glPopClientAttrib();

		// If we bound before, we need to put xplane back where it was
#if IBM
		if(glBindBufferARB)
#endif
			glBindBufferARB(GL_ARRAY_BUFFER_ARB, xpBuffer);

		obj->lods[lodIdx].triangleList.clear();
		obj->lods[lodIdx].pointPool.Purge();
	}
	glCallList(obj->lods[lodIdx].dl);
}

/*****************************************************
			Textured Lights Drawing

 Draw one or more lights on our OBJ.
 RGB of 11,11,11 is a RED NAV light
 RGB of 22,22,22 is a GREEN NAV light
 RGB of 33,33,33 is a Red flashing BEACON light
 RGB of 44,44,44 is a White flashing STROBE light
 RGB of 55,55,55 is a landing light
 RGB of 66,66,66 is a taxi light
******************************************************/
void	OBJ_BeginLightDrawing()
{
	sFOV = XPLMGetDataf(sFOVRef);

	// Setup OpenGL for the drawing
	XPLMSetGraphicsState(1, 1, 0,   1, 1 ,   1, 0);
	XPLMBindTexture2d(sLightTexture, 0);
}

void	OBJ_DrawLights(XPMPPlane_t *plane, float inDistance, double inX, double inY,
					   double inZ, double inPitch, double inRoll, double inHeading,
					   xpmp_LightStatus lights)
{
	bool navLights = lights.navLights == 1;
	bool bcnLights = lights.bcnLights == 1;
	bool strbLights = lights.strbLights == 1;
	bool landLights = lights.landLights == 1;
	bool taxiLights = lights.taxiLights == 1;

	if (! plane->objHandle) { return; }
	auto obj = plane->objHandle.get();
	int offset = lights.timeOffset;

	// flash frequencies
	if(bcnLights) {
		bcnLights = false;
		int x = (int)(XPLMGetElapsedTime() * 1000 + offset) % 1200;
		switch(lights.flashPattern) {
		case xpmp_Lights_Pattern_EADS:
			// EADS pattern: two flashes every 1.2 seconds
			if(x < 120 || ((x > 240 && x < 360))) bcnLights = true;
			break;

		case xpmp_Lights_Pattern_GA:
			// GA pattern: 900ms / 1200ms
			if((((int)(XPLMGetElapsedTime() * 1000 + offset) % 2100) < 900)) bcnLights = true;
			break;

		case xpmp_Lights_Pattern_Default:
		default:
			// default pattern: one flash every 1.2 seconds
			if(x < 120) bcnLights = true;
			break;
		}

	}
	if(strbLights) {
		strbLights = false;
		int x = (int)(XPLMGetElapsedTime() * 1000 + offset) % 1700;
		switch(lights.flashPattern) {
		case xpmp_Lights_Pattern_EADS:
			if(x < 80 || (x > 260 && x < 340)) strbLights = true;
			break;

		case xpmp_Lights_Pattern_GA:
			// similar to the others.. but a little different frequency :)
			x = (int)(XPLMGetElapsedTime() * 1000 + offset) % 1900;
			if(x < 100) strbLights = true;
			break;

		case xpmp_Lights_Pattern_Default:
		default:
			if(x < 80) strbLights = true;
			break;
		}
	}

	// Find out what LOD we need to draw
	int lodIdx = -1;
	for(size_t n = 0; n < obj->lods.size(); n++)
	{
		if((inDistance >= obj->lods[n].nearDist) &&
				(inDistance <= obj->lods[n].farDist))
		{
			lodIdx = static_cast<int>(n);
			break;
		}
	}
	// If we didn't find a good LOD bin, we don't draw!
	if(lodIdx == -1)
		return;

	GLfloat size;
	// Where are we looking?
	XPLMCameraPosition_t cameraPos;
	XPLMReadCameraPosition(&cameraPos);
	
	// We can have 1 or more lights on each aircraft
	for (size_t n = 0; n < obj->lods[lodIdx].lights.size(); n++)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		// First we translate to our coordinate system and move the origin
		// to the center of our lights.
		glTranslatef(obj->lods[lodIdx].lights[n].xyz[0],
			obj->lods[lodIdx].lights[n].xyz[1],
			obj->lods[lodIdx].lights[n].xyz[2]);

		// Now we undo the rotation of the plane
		glRotated(-inRoll, 0.0, 0.0, -1.0);
		glRotated(-inPitch, 1.0, 0.0, 0.0);
		glRotated(-inHeading, 0.0, -1.0, 0.0);

		// Now we undo the rotation of the camera
		// NOTE: The order and sign of the camera is backwards
		// from what we'd expect (the plane rotations) because
		// the camera works backwards. If you pan right, everything
		// else moves left!
		glRotated(cameraPos.heading, 0.0, -1.0, 0.0);
		glRotated(cameraPos.pitch, 1.0, 0.0, 0.0);
		glRotated(cameraPos.roll, 0.0, 0.0, -1.0);

		// Find our distance from the camera
		float dx = cameraPos.x - static_cast<float>(inX);
		float dy = cameraPos.y - static_cast<float>(inY);
		float dz = cameraPos.z - static_cast<float>(inZ);
		double distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));

		// Convert to NM
		distance *= kMetersToNM;

		// Scale based on our FOV and Zoom. I did my initial
		// light adjustments at a FOV of 60 so thats why
		// I divide our current FOV by 60 to scale it appropriately.
		distance *= sFOV / 60.0;
		distance /= cameraPos.zoom;

		// Calculate our light size. This is piecewise linear. I noticed
		// that light size changed more rapidly when closer than 3nm so
		// I have a separate equation for that.
		if (distance <= 3.6)
			size = (10.0f * static_cast<GLfloat>(distance)) + 1.0f;
		else
			size = (6.7f * static_cast<GLfloat>(distance)) + 12.0f;

		// Finally we can draw our lights
		// Red Nav
		glBegin(GL_QUADS);
		if ((obj->lods[lodIdx].lights[n].rgb[0] == 11) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 11) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 11))
		{
			if (navLights) {
				glColor4fv(kNavLightRed);
				glTexCoord2f(0.0f, 0.5f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.0f, 1.0f); glVertex2f(-(size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 1.0f); glVertex2f((size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f((size / 2.0f), -(size / 2.0f));
			}
		}
		// Green Nav
		else if ((obj->lods[lodIdx].lights[n].rgb[0] == 22) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 22) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 22))
		{
			if (navLights) {
				glColor4fv(kNavLightGreen);
				glTexCoord2f(0.0f, 0.5f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.0f, 1.0f); glVertex2f(-(size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 1.0f); glVertex2f((size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f((size / 2.0f), -(size / 2.0f));
			}
		}
		// Beacon
		else if ((obj->lods[lodIdx].lights[n].rgb[0] == 33) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 33) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 33))
		{
			if (bcnLights)
			{
				glColor4fv(kNavLightRed);
				glTexCoord2f(0.0f, 0.5f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.0f, 1.0f); glVertex2f(-(size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 1.0f); glVertex2f((size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f((size / 2.0f), -(size / 2.0f));
			}
		}
		// Strobes
		else if ((obj->lods[lodIdx].lights[n].rgb[0] == 44) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 44) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 44))
		{
			if (strbLights)
			{
				glColor4fv(kStrobeLight);
				glTexCoord2f(0.25f, 0.0f); glVertex2f(-(size / 1.5f), -(size / 1.5f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f(-(size / 1.5f), (size / 1.5f));
				glTexCoord2f(0.50f, 0.5f); glVertex2f((size / 1.5f), (size / 1.5f));
				glTexCoord2f(0.50f, 0.0f); glVertex2f((size / 1.5f), -(size / 1.5f));
			}
		}
		// Landing Lights
		else if ((obj->lods[lodIdx].lights[n].rgb[0] == 55) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 55) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 55))
		{
			if (landLights) {
				// BEN SEZ: modulate the _alpha to make this dark, not
				// the light color.  Otherwise if the sky is fairly light the light
				// will be darker than the sky, which looks f---ed during the day.
				float color[4];
				color[0] = kLandingLight[0];
				if (color[0] < 0.0) color[0] = 0.0;
				color[1] = kLandingLight[1];
				if (color[0] < 0.0) color[0] = 0.0;
				color[2] = kLandingLight[2];
				if (color[0] < 0.0) color[0] = 0.0;
				color[3] = kLandingLight[3] * ((static_cast<float>(distance) * -0.05882f) + 1.1764f);
				glColor4fv(color);
				glTexCoord2f(0.25f, 0.0f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f(-(size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.50f, 0.5f); glVertex2f((size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.50f, 0.0f); glVertex2f((size / 2.0f), -(size / 2.0f));
			}
		}
		// taxi lights
		else if ((obj->lods[lodIdx].lights[n].rgb[0] == 66) &&
			(obj->lods[lodIdx].lights[n].rgb[1] == 66) &&
			(obj->lods[lodIdx].lights[n].rgb[2] == 66))
		{
			if (taxiLights) {
				float color[4];
				color[0] = kTaxiLight[0];
				if (color[0] < 0.0) color[0] = 0.0;
				color[1] = kTaxiLight[1];
				if (color[1] < 0.0) color[1] = 0.0;
				color[2] = kTaxiLight[2];
				if (color[2] < 0.0) color[2] = 0.0;
				color[3] = kTaxiLight[3] * ((static_cast<float>(distance) * -0.05882f) + 1.1764f);
				glColor4fv(color);
				glTexCoord2f(0.25f, 0.0f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f(-(size / 2.0f), -(size / 2.0f));
				glTexCoord2f(0.50f, 0.5f); glVertex2f((size / 2.0f), (size / 2.0f));
				glTexCoord2f(0.50f, 0.0f); glVertex2f((size / 2.0f), -(size / 2.0f));
			}
		} else {
			// rear nav light and others? I guess...
			if(navLights) {
				glColor3f(
						obj->lods[lodIdx].lights[n].rgb[0] * 0.1f,
						obj->lods[lodIdx].lights[n].rgb[1] * 0.1f,
						obj->lods[lodIdx].lights[n].rgb[2] * 0.1f);
				glTexCoord2f(0.0f, 0.5f); glVertex2f(-(size/2.0f), -(size/2.0f));
				glTexCoord2f(0.0f, 1.0f); glVertex2f(-(size/2.0f), (size/2.0f));
				glTexCoord2f(0.25f, 1.0f); glVertex2f((size/2.0f), (size/2.0f));
				glTexCoord2f(0.25f, 0.5f); glVertex2f((size/2.0f), -(size/2.0f));
			}
		}
		glEnd();
		// Put OpenGL back how we found it
		glPopMatrix();
	}
}

int		OBJ_GetModelTexID(int model)
{
	if (model >= static_cast<int>(sObjects.size())) { return 0; }
	else { return sObjects[model].texnum; }
}

string OBJ_GetLitTextureByTexture(const std::string &texturePath)
{
	static const vector<string> extensions =
	{
		"LIT"
	};
	static const string defaultExtension("_LIT");

	auto position = texturePath.find_last_of('.');
	if(position == std::string::npos) { return {}; }

	for (const auto &extension : extensions)
	{
		string textureLitPath = texturePath;
		textureLitPath.insert(position, extension);

		// Does the file exist?
		if(DoesFileExist(textureLitPath)) { return textureLitPath; }
	}

	// If none of them exist, we return the default "_LIT" without testing.
	// If loading fails later, the user will be properly informed.
	string textureLitPath = texturePath;
	textureLitPath.insert(position, defaultExtension);
	return textureLitPath;
}
