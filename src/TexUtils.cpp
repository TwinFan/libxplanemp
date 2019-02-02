/* 
 * Copyright (c) 2006, Laminar Research.
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
#include "XPLMGraphics.h"
#include "TexUtils.h"
#include "XPLMUtilities.h"
#include "XOGLUtils.h"
#include "XPMPMultiplayerVars.h"
#include <utility>
#include <algorithm>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <XPMPMultiplayer.h>
#include <XPLMDataAccess.h>
#if IBM
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#elif APL
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

using std::swap;

float	xpmp_tex_maxAnisotropy = 1.0;
bool	xpmp_tex_useAnisotropy = false;
int 	xpmp_tex_maxSize = 1024;

static void HalfBitmap(ImageInfo& ioImage)
{
	int row_b = (ioImage.width * ioImage.channels) + ioImage.pad;
	
	const unsigned char * srcp1 = ioImage.bitmap.data();
	const unsigned char * srcp2 = ioImage.bitmap.data() + row_b;
	unsigned char * dstp = ioImage.bitmap.data();
	
	ioImage.height /= 2;
	ioImage.width /= 2;
	int yr = ioImage.height;

	if (ioImage.channels == 3)
	{
		while (yr--)
		{
			int xr = ioImage.width;
			while (xr--)
			{
				int t1  = *srcp1++;
				int t2  = *srcp1++;
				int t3  = *srcp1++;

				t1 += *srcp1++;
				t2 += *srcp1++;
				t3 += *srcp1++;

				t1 += *srcp2++;
				t2 += *srcp2++;
				t3 += *srcp2++;

				t1 += *srcp2++;
				t2 += *srcp2++;
				t3 += *srcp2++;

				t1 >>= 2;
				t2 >>= 2;
				t3 >>= 2;

				*dstp++ = t1;
				*dstp++ = t2;
				*dstp++ = t3;
			}
			
			srcp1 += row_b;
			srcp1 += ioImage.pad;
			srcp2 += row_b;
			srcp2 += ioImage.pad;
		}
	} else {

		while (yr--)
		{
			int xr = ioImage.width;
			while (xr--)
			{
				int t1  = *srcp1++;
				int t2  = *srcp1++;
				int t3  = *srcp1++;
				int t4  = *srcp1++;

				t1 += *srcp1++;
				t2 += *srcp1++;
				t3 += *srcp1++;
				t4 += *srcp1++;

				t1 += *srcp2++;
				t2 += *srcp2++;
				t3 += *srcp2++;
				t4 += *srcp2++;

				t1 += *srcp2++;
				t2 += *srcp2++;
				t3 += *srcp2++;
				t4 += *srcp2++;

				t1 >>= 2;
				t2 >>= 2;
				t3 >>= 2;
				t4 >>= 2;

				*dstp++ = t1;
				*dstp++ = t2;
				*dstp++ = t3;
				*dstp++ = t4;
			}
			
			srcp1 += row_b;
			srcp1 += ioImage.pad;
			srcp2 += row_b;
			srcp2 += ioImage.pad;
		}
	}
	ioImage.pad = 0;

}

bool LoadTextureFromFile(const std::string &inFileName, bool magentaAlpha, bool inWrap, bool inMipmap, int inDeres,
						 GLuint *outTexNum, int *outWidth, int *outHeight)
{
	struct ImageInfo	im;
	if (!LoadImageFromFile(inFileName, magentaAlpha, inDeres, im, outWidth, outHeight)) { 
		return false; 
	}
	if (!VerifyTextureImage(inFileName, im)) {
		return false;
	}
	return LoadTextureFromMemory(im, magentaAlpha, inWrap, inMipmap, *outTexNum);
}


bool LoadImageFromFile(const std::string &inFileName, bool magentaAlpha, int inDeres, ImageInfo &im, int * outWidth, int * outHeight)
{
	if (inFileName.empty()) { return false; }

	int result = CreateBitmapFromPNG(inFileName.c_str(), &im);
	if (result) result = CreateBitmapFromFile(inFileName.c_str(), &im);

	bool ok = false;
	if (result == 0)
	{
		while (inDeres > 0)
		{
			HalfBitmap(im);
			--inDeres;
		}

		if (!magentaAlpha || ConvertBitmapToAlpha(&im) == 0)
		{
			if (im.pad == 0)
			{
				int increment = 3;
				if (magentaAlpha) increment = 4;
				unsigned char *p = im.bitmap.data();
				int count = im.width * im.height;
				while (count--)
				{
					std::swap(p[0], p[2]);
					p += increment;
				}

				if (outWidth) *outWidth = im.width;
				if (outHeight) *outHeight = im.height;
				ok = true;
			}
		}
	}
	return ok;
}

/*
   basically a BSF (x86) - find the lowest bit set - this is so we can reverse it to do
   a Non-power-of-two check 
 */
inline int
purec_find_first_bit_set(unsigned int x)
{
	int r = -1;
	for (unsigned i = 0; i < sizeof(x)*8; i++) {
		if ((1<<(i)) & x) {
			r = i;
		}
	}
	return r;
}

#if defined(_MSC_VER)
	#if _MSC_VER >= 1900
		#include <intrin.h>

inline int
find_first_bit_set(unsigned int x)
{
	DWORD rv = -1;
	_BitScanForward(&rv, static_cast<DWORD>(x));
	return rv;
}
	#else
		#define find_first_bit_set(x) purec_find_first_bit_set(x)
	#endif
#elif defined(__GNUC__)
inline int
find_first_bit_set(unsigned int x)
{
	unsigned int out = 0;

	asm ("bsfl %[x], %[out]"
	 	: [out] "=r" (out)
	 	: [x] "rm" (x) );

	return out;
}
#else
	#define find_first_bit_set(x) purec_find_first_bit_set(x)
#endif

/*
  Preflight an image to ensure it's a valid texture before we even attempt a LoadTextureFromMemory.
  the main things that came from elsewhere that we can die horribly on are:

  * non-power-of-two texture lengths (not technically required anymore, but this will probably 
  *       break a bunch of other things in our code)
  * excessive texture sizes (sides must be less than GL_MAX_TEXTURE_SIZE)
  * sizes < 0 (should never happen)

  nothing else we do should cause an error that we can't guard against, and all of these checks can 
  take place asynchronously.
*/
bool
VerifyTextureImage(const string &filename, const ImageInfo &im)
{
	stringstream errout;
	if (im.width <= 0 || im.height <= 0) {
		errout << XPMP_CLIENT_NAME": Texture " << filename << " failed vailidation - dimensions ("
			<< im.width << " x " << im.height << ") <= 0" << endl;
		XPLMDebugString(errout.str().c_str());
		return false;
	}
	if (im.width > xpmp_tex_maxSize || im.height > xpmp_tex_maxSize) {
		errout << XPMP_CLIENT_NAME": Texture " << filename << " failed validation - dimensions ("
			<< im.width << " x " << im.height << ") exceeds GL maximum ("
			<< xpmp_tex_maxSize << ")" << endl;
		XPLMDebugString(errout.str().c_str());
		return false;
	}
	// validate NPOT.
	int wpow = find_first_bit_set(im.width);
	// wpow can't be -1 becasue we checked for <=0 already.
	if (im.width != (1<<(wpow)))  {
		errout << XPMP_CLIENT_NAME": Texture " << filename << "failed validation - width ("
			<< im.width << ") is not a power of two" << endl;			
		XPLMDebugString(errout.str().c_str());
		return false;
	}
	int hpow = find_first_bit_set(im.height);
	// hpow can't be -1 becasue we checked for <=0 already.
	if (im.height != (1<<(hpow)))  {
		errout << XPMP_CLIENT_NAME": Texture " << filename << "failed validation - height ("
			<< im.height << ") is not a power of two" << endl;			
		XPLMDebugString(errout.str().c_str());
		return false;
	}
	return true;
}


#ifdef DEBUG_GL
extern void XPMPSetupGLDebug();
#endif

static XPLMDataRef sAnisotropicLevel = nullptr;

bool LoadTextureFromMemory(ImageInfo &im, bool magentaAlpha, bool inWrap, bool mipmap, GLuint &texNum)
{
	float	tex_anisotropyLevel = gFloatPrefsFunc("planes", "texture_anisotropy", 0.0);
	if (sAnisotropicLevel == nullptr) {
		sAnisotropicLevel = XPLMFindDataRef("sim/private/controls/reno/aniso_filter");
	}
	if (sAnisotropicLevel != nullptr) {
		tex_anisotropyLevel = static_cast<GLfloat>(XPLMGetDatai(sAnisotropicLevel));
	}
	if (tex_anisotropyLevel > xpmp_tex_maxAnisotropy) {
		tex_anisotropyLevel = xpmp_tex_maxAnisotropy;
	}


#ifdef DEBUG_GL
	OGLDEBUG(glPushDebugGroup(GL_DEBUG_SOURCE_THIRD_PARTY, XPMP_DBG_TexLoad, -1, "LoadTextureFromMemory"));
	OGLDEBUG(XPMPSetupGLDebug());

	// we must use glGetError to force the error queue back into a known state before we can use it for anything meaningful below.
	static bool dirtyGLState = false;
	static bool dirtyGLStateReported = false;
	while (GL_NO_ERROR != glGetError())
		dirtyGLState = true;
	if (dirtyGLState && !dirtyGLStateReported) {
		XPLMDebugString(XPMP_CLIENT_NAME": GL Error State was bad upon texture load (will not be reported again)\n");
		dirtyGLStateReported = true;
	}
#endif /* #ifdef DEBUG_GL */

	if (texNum == 0) { XPLMGenerateTextureNumbers(reinterpret_cast<int *>(&texNum), 1); }
#ifdef DEBUG_GL
	bool texNumError = false;
	while (GL_NO_ERROR != glGetError()) {
		texNumError = true;
	}
	if (texNumError)
	{
		XPLMDebugString(XPMP_CLIENT_NAME " Couldn't generate texture number.");
		texNum = 0;
		OGLDEBUG(glPopDebugGroup());
		return false;
	}
#endif /* #ifdef DEBUG_GL */


	if (!magentaAlpha || ConvertBitmapToAlpha(&im) == 0)
	{
		if (im.pad == 0)
		{
			XPLMBindTexture2d(texNum, 0);

			
			OGLDEBUG(glDebugMessageInsert(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, XPMP_DBG_TexLoad_CreateTex, GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Creating Texture"));
			if (magentaAlpha)
			{
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, im.width ,im.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, im.bitmap.data());
			}
			else
			{
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, im.width ,im.height, 0, GL_RGB, GL_UNSIGNED_BYTE, im.bitmap.data());
			}

#ifdef DEBUG_GL
			bool texGenError = false;
			GLenum	tErr;
			while ((tErr = glGetError()) != GL_NO_ERROR) {
				std::stringstream	msgOut;
				msgOut << XPMP_CLIENT_NAME ": Failed to create texture: 0x" << std::ios::hex << tErr << std::endl;				
				XPLMDebugString(msgOut.str().c_str());
				texGenError = true;
			}
			if (texGenError) {
				// asume no textures resources used because it was the glTexImage2D that failed, and bail
				texNum = 0;
				OGLDEBUG(glPopDebugGroup());
				return false;
			}
#endif /* #ifdef DEBUG_GL */

			if (mipmap) {
				OGLDEBUG(glDebugMessageInsert(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, XPMP_DBG_TexLoad_GenMips, GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Generating Mipmaps"));
				// https://www.khronos.org/opengl/wiki/Common_Mistakes#Automatic_mipmap_generation:
				// It has been reported that on some ATI drivers, glGenerateMipmap(GL_TEXTURE_2D)
				// has no effect unless you precede it with a call to glEnable(GL_TEXTURE_2D) in this particular case.
				if (xpmp_tex_useAnisotropy && tex_anisotropyLevel > 1.0) {
					glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, tex_anisotropyLevel);
				}
				glEnable(GL_TEXTURE_2D);
				glGenerateMipmap(GL_TEXTURE_2D);
			}
		}
	}

	DestroyBitmap(im);

	OGLDEBUG(glDebugMessageInsert(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, XPMP_DBG_TexLoad, GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Configuring Filtering"));

	// BAS note: for some reason on my WinXP system with GF-FX, if
	// I do not set these explicitly to linear, I get no drawing at all.
	// Who knows what default state the card is in. :-(
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);

	static const char * ver_str = (const char *) glGetString(GL_VERSION);
	static const char * ext_str = (const char *) glGetString(GL_EXTENSIONS);

	static bool tex_clamp_avail =
			strstr(ext_str,"GL_SGI_texture_edge_clamp"		) ||
			strstr(ext_str,"GL_SGIS_texture_edge_clamp"		) ||
			strstr(ext_str,"GL_ARB_texture_edge_clamp"		) ||
			strstr(ext_str,"GL_EXT_texture_edge_clamp"		) ||
			strncmp(ver_str,"1.2", 3) ||
			strncmp(ver_str,"1.3", 3) ||
			strncmp(ver_str,"1.4", 3) ||
			strncmp(ver_str,"1.5", 3);

	OGLDEBUG(glDebugMessageInsert(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, XPMP_DBG_TexLoad, GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Configuring Texture Wrap"));


	if(inWrap 		   ){glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT		 );
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT		 );}
	else if(tex_clamp_avail){glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);}
	else					{glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP		 );
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP		 );}

	OGLDEBUG(glDebugMessageInsert(GL_DEBUG_SOURCE_THIRD_PARTY, GL_DEBUG_TYPE_MARKER, XPMP_DBG_TexLoad, GL_DEBUG_SEVERITY_NOTIFICATION, -1, "Finished Load - checking errors"));
#ifdef DEBUG_GL
	int err = glGetError();
	if (err)
	{
		char buf[256];
		sprintf(buf, "Texture load got OGL err: %x\n", err);
		XPLMDebugString(buf);
		// flush any texture resources being held by the failed load
		GLuint	textures[] = { texNum };
		glDeleteTextures(1, textures);
		// flush any pending error states
		while (GL_NO_ERROR != glGetError())
			;
		texNum = 0;
		OGLDEBUG(glPopDebugGroup());
		return false;
	}
#endif /* #ifdef DEBUG_GL */
	OGLDEBUG(glPopDebugGroup());
	return true;
}
