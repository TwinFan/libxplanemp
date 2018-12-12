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

#include "XPMPMultiplayerCSL.h"
#include "XPLMUtilities.h"
#include "XPMPMultiplayerObj.h"
#include "XStringUtils.h"
#include "XOGLUtils.h"
#include "XUtils.h"
#include <stdio.h>
#include <algorithm>
//#include "PlatformUtils.h"
#include <errno.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <functional>
#include <cctype>

using std::max;

#if APL
#include <Carbon/Carbon.h>
#endif

// Set this to 1 to get TONS of diagnostics on what the lib is doing.
#define 	DEBUG_CSL_LOADING 0

// Set this to 1 to cause AIRLINE and LIVERY to create ICAO codes automatically
#define USE_DEFAULTING 0

enum {
	pass_Depend,
	pass_Load,
	pass_Count
};

/************************************************************************
 * UTILITY ROUTINES
 ************************************************************************/

#if APL

template <typename T>
struct CFSmartPtr {
	CFSmartPtr(T p) : p_(p) {						  }
	~CFSmartPtr()			 { if (p_) CFRelease(p_); }
	operator T ()			 { return p_; }
	T p_;
};

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif

int Posix2HFSPath(const char *path, char *result, int resultLen)
{
	CFSmartPtr<CFStringRef>		inStr(CFStringCreateWithCString(kCFAllocatorDefault, path ,kCFStringEncodingMacRoman));
	if (inStr == NULL) return -1;
	
	CFSmartPtr<CFURLRef>		url(CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLPOSIXPathStyle,0));
	if (url == NULL) return -1;
	
	CFSmartPtr<CFStringRef>		outStr(CFURLCopyFileSystemPath(url, kCFURLHFSPathStyle));
	if (outStr == NULL) return -1;
	
	if (!CFStringGetCString(outStr, result, resultLen, kCFStringEncodingMacRoman))
		return -1;

	return 0;
}

int HFS2PosixPath(const char *path, char *result, int resultLen)
{
	bool is_dir = (path[strlen(path)-1] == ':');

	CFSmartPtr<CFStringRef>		inStr(CFStringCreateWithCString(kCFAllocatorDefault, path ,kCFStringEncodingMacRoman));
	if (inStr == NULL) return -1;
	
	CFSmartPtr<CFURLRef>		url(CFURLCreateWithFileSystemPath(kCFAllocatorDefault, inStr, kCFURLHFSPathStyle,0));
	if (url == NULL) return -1;
	
	CFSmartPtr<CFStringRef>		outStr(CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle));
	if (outStr == NULL) return -1;
	
	if (!CFStringGetCString(outStr, result, resultLen, kCFStringEncodingMacRoman))
		return -1;

	if(is_dir) strcat(result, "/");

	return 0;
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#endif

// trim from start (in place)
static inline void ltrim(std::string &s)
{
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](char c) { return ! std::isspace(c); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
	s.erase(std::find_if(s.rbegin(), s.rend(), [](char c) { return ! std::isspace(c); }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s)
{
	ltrim(s);
	rtrim(s);
}

static void MakePartialPathNativeObj(string& io_str)
{
	//	char sep = *XPLMGetDirectorySeparator();
	for(size_t i = 0; i < io_str.size(); ++i)
		if(io_str[i] == '/' || io_str[i] == ':' || io_str[i] == '\\')
			io_str[i] = '/';
}

struct XPLMDump { 
	XPLMDump() { }
	
	XPLMDump(const string& inFileName, int lineNum, const char * line) {
		XPLMDebugString(XPMP_CLIENT_NAME " WARNING: Parse Error in file ");
		XPLMDebugString(inFileName.c_str());
		XPLMDebugString(" line ");
		char buf[32];
		sprintf(buf,"%d", lineNum);
		XPLMDebugString(buf);
		XPLMDebugString(".\n              ");
		XPLMDebugString(line);
		XPLMDebugString(".\n");
	}

	XPLMDump(const string& inFileName, int lineNum, const string& line) {
		XPLMDebugString(XPMP_CLIENT_NAME " WARNING: Parse Error in file ");
		XPLMDebugString(inFileName.c_str());
		XPLMDebugString(" line ");
		char buf[32];
		sprintf(buf,"%d", lineNum);
		XPLMDebugString(buf);
		XPLMDebugString(".\n              ");
		XPLMDebugString(line.c_str());
		XPLMDebugString(".\n");
	}
	
	XPLMDump& operator<<(const char * rhs) {
		XPLMDebugString(rhs);
		return *this;
	}
	XPLMDump& operator<<(const std::string& rhs) {
		XPLMDebugString(rhs.c_str());
		return *this;
	}
	XPLMDump& operator<<(int n) {
		char buf[255];
		sprintf(buf, "%d", n);
		XPLMDebugString(buf);
		return *this;
	}
	XPLMDump& operator<<(size_t n) {
		char buf[255];
		sprintf(buf, "%u", static_cast<unsigned>(n));
		XPLMDebugString(buf);
		return *this;
	}
};


static 	char * 			fgets_multiplatform(char * s, int n, FILE * file);
static 	void			BreakStringPvt(const char * inString, std::vector<std::string>& outStrings,  int maxBreak, const std::string& inSeparators);
static	bool			DoPackageSub(std::string& ioPath);

bool			DoPackageSub(std::string& ioPath)
{
	for (auto i = gPackages.begin(); i != gPackages.end(); ++i)
	{
		if (strncmp(i->name.c_str(), ioPath.c_str(), i->name.size()) == 0)
		{
			ioPath.erase(0, i->name.size());
			ioPath.insert(0, i->path);
			return true;
		}
	}
	return false;
}


// This routine gets one line, but handles any platforms crlf setup.
char * fgets_multiplatform(char * s, int n, FILE * file)
{
	char *	p = s;

	// Save one slot for the null.  If we do not have enough memory
	// to do this, bail.
	if (--n < 0)
		return(NULL);
	
	// Only bother to read if we have enough space in the char buf.
	if (n)
	{
		int c;
		do
		{
			c = getc(file);
			
			// EOF: this could mean I/O error or end of file.
			if (c == EOF)
			{
				if (feof(file) && p != s)	// We read something and now the file's done, ok.
					break;
				else
				{
					// Haven't read yet?  I/O error?  Return NULL!
					return(NULL);
				}
			}
			
			*p++ = c;
		}
		// Stop when we see either newline char or the buffer is full.
		// Note that the \r\n IS written to the line.
		while (c != '\n' && c != '\r' && --n);

		// Ben's special code: eat a \n if it follows a \r, etc.  Mac stdio
		// swizzles these guys a bit, so we will consolidate BOTH \r\n and \n\r into
		// just the first.
		if (c == '\r')
		{
			int c1 = getc(file);
			if (c1 != '\n') ungetc(c1, file);
		}
		if (c == '\n')
		{
			int c1 = getc(file);
			if (c1 != '\r') ungetc(c1, file);
		}
	}

	// Unless we're bailing with NULL, we MUST null terminate.
	*p = 0;
	
	return(s);
}

// This routine breaks a line into one or more tokens based on delimitors.
void	BreakStringPvt(const char * inString, std::vector<std::string>& outStrings, 
					   int maxBreak, const std::string& inSeparators)
{
	outStrings.clear();
	
	const char * endPos = inString + strlen(inString);
	const char * iter = inString;
	while (iter < endPos)
	{
		while ((iter < endPos) && (inSeparators.find(*iter) != std::string::npos))
			++iter;
		if (iter < endPos)
		{
			if (maxBreak && (maxBreak == static_cast<int>(outStrings.size()+1)))
			{
				outStrings.push_back(std::string(iter, endPos));
				return;
			}
			const char * wordEnd = iter;
			while ((wordEnd < endPos) && (inSeparators.find(*wordEnd) == std::string::npos))
				++wordEnd;
			
			outStrings.push_back(std::string(iter, wordEnd));
			
			iter = wordEnd;
		}
	}
}


/************************************************************************
 * CSL LOADING
 ************************************************************************/

bool			CSL_Init(
		const char* inTexturePath)
{
	obj_init();
	bool ok = OBJ_Init(inTexturePath);
	if (!ok)
		XPLMDump() << XPMP_CLIENT_NAME " WARNING: we failed to find xpmp's custom lighting texture at " << inTexturePath << ".\n";
	return ok;
}

bool ParseExportCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	if (tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line)  << XPMP_CLIENT_NAME " WARNING: EXPORT_NAME command requires 1 argument.\n";
		return false;
	}

	auto p = std::find_if(gPackages.begin(), gPackages.end(), [&tokens](CSLPackage_t p) { return p.name == tokens[1]; } );
if (p == gPackages.end())
{
	package.path = path;
	package.name = tokens[1];
	return true;
}
else
{
XPLMDump(path, lineNum, line)  << XPMP_CLIENT_NAME " WARNING: Package name " << tokens[1].c_str() << " already in use by " << p->path.c_str() << " reqested by use by " << path.c_str() << "'\n";
return false;
}
}

bool ParseDependencyCommand(const std::vector<std::string> &tokens, CSLPackage_t &/*package*/, const string& path, int lineNum, const string& line)
{
	if (tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: DEPENDENCY command needs 1 argument.\n";
		return false;
	}

	if (std::count_if(gPackages.begin(), gPackages.end(), [&tokens](CSLPackage_t p) { return p.name == tokens[1]; }) == 0)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: required package " << tokens[1] << " not found. Aborting processing of this package.\n";
		return false;
	}

	return true;
}

bool ParseObjectCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	std::vector<std::string> dupTokens = tokens;
	BreakStringPvt(line.c_str(), dupTokens, 2, " \t\r\n");
	if (tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: OBJECT command takes 1 argument.\n";
		return false;
	}
	std::string relativePath(tokens[1]);
	MakePartialPathNativeObj(relativePath);
	std::string fullPath(relativePath);
	if (!DoPackageSub(fullPath))
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: package not found.\n";
		return false;
	}

	std::vector<std::string> dirNames;
	BreakStringPvt(relativePath.c_str(), dirNames, 0, "/");
	// Replace the first one being the package name with the package root dir
	std::string packageRootDir = package.path.substr(package.path.find_last_of('/') + 1);
	dirNames[0] = packageRootDir;
	// Remove the last one being the obj itself
	string objFileName = dirNames.back();
	dirNames.pop_back();

	// Remove *.obj extension
	objFileName.erase(objFileName.find_last_of('.'));

	package.planes.push_back(CSLPlane_t());
	package.planes.back().dirNames = dirNames;
	package.planes.back().objectName = objFileName;
	package.planes.back().plane_type = plane_Obj;
	package.planes.back().file_path = fullPath;
	package.planes.back().moving_gear = true;
	package.planes.back().textureName = OBJ_DefaultModel(fullPath);
#if DEBUG_CSL_LOADING
	XPLMDebugString("      Got Object: ");
	XPLMDebugString(fullPath.c_str());
	XPLMDebugString("\n");
#endif

	return true;
}

bool ParseTextureCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	if(tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: TEXTURE command takes 1 argument.\n";
		return false;
	}

	// Load regular texture
	string relativeTexPath = tokens[1];
	MakePartialPathNativeObj(relativeTexPath);
	string absoluteTexPath(relativeTexPath);

	if (!DoPackageSub(absoluteTexPath))
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: package not found.\n";
		return false;
	}

	string textureFilename = absoluteTexPath;
	// Remove directory if present.
	textureFilename.erase(0, textureFilename.find_last_of('/') + 1);
	// Remove extension if present.
	textureFilename.erase(textureFilename.find_last_of('.'));

	package.planes.back().textureName = textureFilename;
	package.planes.back().texturePath = absoluteTexPath;
	package.planes.back().textureLitPath = OBJ_GetLitTextureByTexture(absoluteTexPath);

#if DEBUG_CSL_LOADING
	XPLMDebugString("      Got texture: ");
	XPLMDebugString(absoluteTexPath.c_str());
	XPLMDebugString("\n");
#endif

	return true;
}

bool ParseAircraftCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// AIRCAFT <min> <max> <path>
	if (tokens.size() != 4)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: AIRCRAFT command takes 3 arguments.\n";
	}

	int sim, xplm;
	XPLMHostApplicationID 	host;
	XPLMGetVersions(&sim, &xplm, &host);

	if (sim >= atoi(tokens[1].c_str()) && sim <= atoi(tokens[2].c_str()))
	{
		string relativePath = tokens[3];
		MakePartialPathNativeObj(relativePath);
		string absolutePath(relativePath);
		if (!DoPackageSub(absolutePath))
		{
			XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: package not found.\n";
			return false;
		}
		package.planes.push_back(CSLPlane_t());
		//! \todo Fill in acf model name information
		package.planes.back().plane_type = plane_Austin;
		package.planes.back().file_path = absolutePath;
		package.planes.back().moving_gear = true;
		package.planes.back().austin_idx = -1;
#if DEBUG_CSL_LOADING
		XPLMDebugString("      Got Airplane: ");
		XPLMDebugString(absolutePath.c_str());
		XPLMDebugString("\n");
#endif

	}

	return true;
}

bool ParseObj8AircraftCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// OBJ8_AIRCRAFT <path>
	if (tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: OBJ8_AIRCARFT command takes 1 argument.\n";
	}

	package.planes.push_back(CSLPlane_t());
	package.planes.back().plane_type = plane_Obj8;
	package.planes.back().file_path = tokens[1];
	package.planes.back().moving_gear = true;
	package.planes.back().texID = 0;
	package.planes.back().texLitID = 0;
	package.planes.back().obj_idx = -1;
	return true;
}

bool ParseObj8Command(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// OBJ8 <group> <animate YES|NO> <filename>
	if (tokens.size() != 4)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: OBJ8_AIRCARFT command takes 1 argument.\n";
	}

	// err - obj8 record at stupid place in file
	if(package.planes.empty() || package.planes.back().plane_type != plane_Obj8) return false;

	obj_for_acf		att;

	if(tokens[1] == "GLASS")
		att.draw_type = draw_glass;
	else if(tokens[1] == "LIGHTS")
		att.draw_type = draw_lights;
	else if(tokens[1] == "LOW_LOD")
		att.draw_type = draw_low_lod;
	else if(tokens[1] == "SOLID")
		att.draw_type = draw_solid;
	else {
		// err crap enum
	}

	if(tokens[2] == "YES")
		att.needs_animation = true;
	else if(tokens[2] == "NO")
		att.needs_animation = false;
	else
	{
		// crap flag
	}

	string relativePath = tokens[3];
	MakePartialPathNativeObj(relativePath);
	//! \todo Fill in Obj8 model name information
	string absolutePath(relativePath);
	if (!DoPackageSub(absolutePath))
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: package not found.\n";
		return false;
	}

	char xsystem[1024];
	XPLMGetSystemPath(xsystem);

#if APL
	HFS2PosixPath(xsystem, xsystem, 1024);
#endif

	size_t sys_len = strlen(xsystem);
	if(absolutePath.size() > sys_len)
		absolutePath.erase(absolutePath.begin(),absolutePath.begin() + sys_len);
	else
	{
		// should probaby freak out here.
	}

	att.handle = NULL;
	att.file = absolutePath;

	package.planes.back().attachments.push_back(att);

	return true;
}

bool ParseHasGearCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// HASGEAR YES|NO
	if (tokens.size() != 2 || (tokens[1] != "YES" && tokens[1] != "NO"))
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: HASGEAR takes one argument that must be YES or NO.\n";
		return false;
	}

	if (tokens[1] == "YES")
	{
		package.planes.back().moving_gear = true;
		return true;
	}
	else if (tokens[1] == "NO")
	{
		package.planes.back().moving_gear = false;
		return true;
	}
	else
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: HASGEAR must have a YES or NO argument, but we got " << tokens[1] << ".\n";
		return false;
	}
}

bool ParseIcaoCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// ICAO <code>
	if (tokens.size() != 2)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: ICAO command takes 1 argument.\n";
		return false;
	}

	std::string icao = tokens[1];
	package.planes.back().icao = icao;
	std::string group = gGroupings[icao];
	if (package.matches[match_icao].count(icao) == 0)
		package.matches[match_icao]	   [icao] = static_cast<int>(package.planes.size()) - 1;
	if (!group.empty())
		if (package.matches[match_group].count(group) == 0)
			package.matches[match_group]      [group] = static_cast<int>(package.planes.size()) - 1;

	return true;
}

bool ParseAirlineCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// AIRLINE <code> <airline>
	if (tokens.size() != 3)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: AIRLINE command takes two arguments.\n";
		return false;
	}

	std::string icao = tokens[1];
	package.planes.back().icao = icao;
	std::string airline = tokens[2];
	package.planes.back().airline = airline;
	std::string group = gGroupings[icao];
	if (package.matches[match_icao_airline].count(icao + " " + airline) == 0)
		package.matches[match_icao_airline]      [icao + " " + airline] = static_cast<int>(package.planes.size()) - 1;
#if USE_DEFAULTING
	if (package.matches[match_icao		].count(icao				) == 0)
		package.matches[match_icao		]      [icao				] = package.planes.size() - 1;
#endif
	if (!group.empty())
	{
#if USE_DEFAULTING
		if (package.matches[match_group	     ].count(group				  ) == 0)
			package.matches[match_group	     ]		[group				  ] = package.planes.size() - 1;
#endif
		if (package.matches[match_group_airline].count(group + " " + airline) == 0)
			package.matches[match_group_airline]		[group + " " + airline] = static_cast<int>(package.planes.size()) - 1;
	}

	return true;
}

bool ParseLiveryCommand(const std::vector<std::string> &tokens, CSLPackage_t &package, const string& path, int lineNum, const string& line)
{
	// LIVERY <code> <airline> <livery>
	if (tokens.size() != 4)
	{
		XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: LIVERY command takes two arguments.\n";
		return false;
	}

	std::string icao = tokens[1];
	package.planes.back().icao = icao;
	std::string airline = tokens[2];
	package.planes.back().airline = airline;
	std::string livery = tokens[3];
	package.planes.back().livery = livery;
	std::string group = gGroupings[icao];
#if USE_DEFAULTING
	if (package.matches[match_icao				].count(icao							   ) == 0)
		package.matches[match_icao				]	   [icao							   ] = package.planes.size() - 1;
	if (package.matches[match_icao				].count(icao							   ) == 0)
		package.matches[match_icao_airline 		]	   [icao + " " + airline			   ] = package.planes.size() - 1;
#endif
	if (package.matches[match_icao_airline_livery ].count(icao + " " + airline + " " + livery) == 0)
		package.matches[match_icao_airline_livery ]	   [icao + " " + airline + " " + livery] = static_cast<int>(package.planes.size()) - 1;
	if (!group.empty())
	{
#if USE_DEFAULTING
		if (package.matches[match_group		 		 ].count(group							     ) == 0)
			package.matches[match_group		 		 ]		[group							     ] = package.planes.size() - 1;
		if (package.matches[match_group_airline		 ].count(group + " " + airline			     ) == 0)
			package.matches[match_group_airline		 ]		[group + " " + airline			     ] = package.planes.size() - 1;
#endif
		if (package.matches[match_group_airline_livery ].count(group + " " + airline + " " + livery) == 0)
			package.matches[match_group_airline_livery ]		[group + " " + airline + " " + livery] = static_cast<int>(package.planes.size()) - 1;
	}

	return true;
}

bool ParseVertOffsetCommand(const std::vector<std::string> & tokens,
                            CSLPackage_t& package,
                            const string& path, int lineNum, const string& line)
{
    // VERT_OFFSET <offset.decimal>
    if (tokens.size() != 2)
    {
        XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: VERT_OFFSET command takes one argument: the offset.\n";
        return false;
    }
    
    // just try converting to float, then some sanity checks
    float vertOffset = (float)atof(tokens[1].c_str());
    if (vertOffset < -30 || vertOffset > 30) {
        // offset of more than 30m seems a bit unlikely...
        XPLMDump(path, lineNum, line) << XPMP_CLIENT_NAME " WARNING: VERT_OFFSET of more than 30m ignored.\n";
    }
    
    // save vertical offset to current plane
    package.planes.back().vertOffset = vertOffset;
    return true;
}
 
bool ParseDummyCommand(const std::vector<std::string> & /* tokens */, CSLPackage_t & /* package */, const string& /* path */, int /*lineNum*/, const string& /*line*/)
{
	return true;
}

std::string GetFileContent(const std::string &filename)
{
	std::string content;
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in)
	{
		content = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	}
	return content;
}

CSLPackage_t ParsePackageHeader(const string& path, const string& content)
{
	using command = std::function<bool(const std::vector<std::string> &, CSLPackage_t &, const string&, int, const string&)>;

	static const std::map<std::string, command> commands
	{
		{ "EXPORT_NAME", &ParseExportCommand }
	};

	CSLPackage_t package;
	stringstream sin(content);
	if (!sin.good()) { return package; }

	std::string line;
	int lineNum = 0;

	while (std::getline(sin, line))
	{
		++lineNum;
		auto tokens = tokenize(line, " \t\r\n");
		if (!tokens.empty())
		{
			auto it = commands.find(tokens[0]);
			if (it != commands.end())
			{
				bool result = it->second(tokens, package, path, lineNum, line);
				// Stop loop once we found EXPORT command
				if (result) break;
			}
		}
	}

	return package;
}


void ParseFullPackage(const std::string &content, CSLPackage_t &package)
{
	using command = std::function<bool(const std::vector<std::string> &, CSLPackage_t &, const string&, int, const string&)>;

	static const std::map<std::string, command> commands
	{
		{ "EXPORT_NAME", &ParseDummyCommand },
		{ "DEPENDENCY", &ParseDependencyCommand },
		{ "OBJECT", &ParseObjectCommand },
		{ "TEXTURE", &ParseTextureCommand },
		{ "AIRCRAFT", &ParseAircraftCommand },
		{ "OBJ8_AIRCRAFT", &ParseObj8AircraftCommand },
		{ "OBJ8", &ParseObj8Command },
		{ "HASGEAR", &ParseHasGearCommand },
		{ "ICAO", &ParseIcaoCommand },
		{ "AIRLINE", &ParseAirlineCommand },
		{ "LIVERY", &ParseLiveryCommand },
/*** INSERTED by TwinFan ***/
        { "VERT_OFFSET", &ParseVertOffsetCommand },
/*** END OF INSERT ***/
	};

	stringstream sin(content);
	if (!sin.good()) { return; } // exit if file not found

	std::string packageFilePath(package.path);
	packageFilePath += "/";
	packageFilePath += "xsb_aircraft.txt";

	std::string line;
	int lineNum = 0;
	while (std::getline(sin, line))
	{
		++lineNum;
		trim(line);
		if (line.empty() || line[0] == '#') continue;
		auto tokens = tokenize(line, " \t\r\n");
		if (!tokens.empty())
		{
			auto it = commands.find(tokens[0]);
			if (it != commands.end())
			{
				it->second(tokens, package, packageFilePath, lineNum, line);
			}
			else
			{
				XPLMDump(packageFilePath, lineNum, line);
			}
		}
	}
}

bool isPackageAlreadyLoaded(const std::string &packagePath)
{
	bool alreadyLoaded = false;
	for (const auto &package : gPackages)
	{
		if(package.path == packagePath)
		{
			alreadyLoaded = true;
			break;
		}
	}
	return alreadyLoaded;
}

// Assign reasonable default values for vertOffset based on Doc8643
void CSL_DefaultVertOffset(CSLPackage_t& package)
{
    // loop over plane in package
    for (CSLPlane_t& plane: package.planes) {
        // no vertOffset read from file?
        if (plane.vertOffset <= 0) {
            // lookup categorization in Doc8643
            map<string, CSLAircraftCode_t>::const_iterator iterAcCodes =
            gAircraftCodes.find(plane.icao);
            if (iterAcCodes != gAircraftCodes.cend()) {
                // found an Doc8643 element
                const std::string& equip = iterAcCodes->second.equip;
                if (equip.size() == 3) {
                    int nEng = equip[1] - '0';          // number of engines
                    enum { EQUIP_OTHER, EQUIP_JET, EQUIP_TURBO } eType =
                    equip[2] == 'J' ? EQUIP_JET :       // type of engines
                    equip[2] == 'T' ? EQUIP_TURBO : EQUIP_OTHER;
    
                    plane.vertOffset =
                    nEng > 4 ?                          5.5 :   // large planes
                    nEng == 3 && eType == EQUIP_JET ?   5   :   // 3 engine jets
                    nEng == 3 ?                         3   :   // other 3 engine planes
                    nEng == 2 && eType == EQUIP_JET ?   3.5 :   // 2 engine jets
                    nEng == 2 && eType == EQUIP_TURBO ? 1.7 :   // 2 engine turbo props
                    nEng == 2 ?                         1.5 :   // other 2 engine planes
                                                        1.2;    // default, covers small planes
                }
            }
        }
    }
}

// This routine loads the related.txt file and also all packages.
bool CSL_LoadCSL(const char * inFolderPath, const char * inRelatedFile, const char * inDoc8643)
{
	bool ok = true;

	// read the list of aircraft codes
	FILE * aircraft_fi = fopen(inDoc8643, "r");

	if (gIntPrefsFunc("debug", "model_matching", 0))
		XPLMDebugString(string(string(inDoc8643) + " returned " + (aircraft_fi ? "valid" : "invalid") + " fp\n").c_str());
	
	if (aircraft_fi)
	{
		char	buf[1024];
		while (fgets_multiplatform(buf, sizeof(buf), aircraft_fi))
		{
			vector<string>	tokens;
			BreakStringPvt(buf, tokens, 0, "\t\r\n");

			/*
			if (gIntPrefsFunc("debug", "model_matching", 0)) {
				char str[20];
				sprintf(str, "size: %i", tokens.size());
				string s = string(str) + string(": ") + buf;
				XPLMDebugString(s.c_str());
			}
			*/

			// Sample line. Fields are separated by tabs
			// ABHCO	SA-342 Gazelle 	GAZL	H1T	-

			if(tokens.size() < 5) continue;
			CSLAircraftCode_t entry;
			entry.icao = tokens[2];
			entry.equip = tokens[3];
			entry.category = tokens[4][0];

			// Debugging stuff
			/*
			if (gIntPrefsFunc("debug", "model_matching", 0)) {
				XPLMDebugString("Loaded entry: icao code ");
				XPLMDebugString(entry.icao.c_str());
				XPLMDebugString(" equipment ");
				XPLMDebugString(entry.equip.c_str());
				XPLMDebugString(" category ");
				switch(entry.category) {
					case 'L': XPLMDebugString(" light"); break;
					case 'M': XPLMDebugString(" medium"); break;
					case 'H': XPLMDebugString(" heavy"); break;
					default: XPLMDebugString(" other"); break;
				}
				XPLMDebugString("\n");
			}
			*/

			gAircraftCodes[entry.icao] = entry;
		}
		fclose(aircraft_fi);
	} else {
		XPLMDump() << XPMP_CLIENT_NAME " WARNING: could not open ICAO document 8643 at " << inDoc8643 << "\n";
		ok = false;
	}

	// First grab the related.txt file.
	FILE * related_fi = fopen(inRelatedFile, "r");
	if (related_fi)
	{
		char	buf[1024];
		while (fgets_multiplatform(buf, sizeof(buf), related_fi))
		{
			if (buf[0] != ';')
			{
				vector<string>	tokens;
				BreakStringPvt(buf, tokens, 0, " \t\r\n");
				string	group;
				for (size_t n = 0; n < tokens.size(); ++n)
				{
					if (n != 0) group += " ";
					group += tokens[n];
				}
				for (size_t n = 0; n < tokens.size(); ++n)
				{
					gGroupings[tokens[n]] = group;
				}
			}
		}
		fclose(related_fi);
	} else {
		XPLMDump() << XPMP_CLIENT_NAME " WARNING: could not open related.txt at " << inRelatedFile << "\n";
		ok = false;
	}

	// Iterate through all directories using the XPLM and load them.

	char *	name_buf = (char *) malloc(16384);
	char ** index_buf = (char **) malloc(65536);
	int	total, ret;
	
	char folder[1024];

#if APL
	Posix2HFSPath(inFolderPath, folder, sizeof(folder));
#else
	strcpy(folder,inFolderPath);
#endif
	XPLMGetDirectoryContents(folder, 0, name_buf, 16384, index_buf, 65536 / sizeof(char*),
							 &total, &ret);

	vector<string>	pckgs;
	for (int r = 0; r < ret; ++r)
	{
#if APL
		if (index_buf[r][0] == '.')
			continue;
#endif	
		char * foo = index_buf[r];
		string	path(inFolderPath);
		path += "/";//XPLMGetDirectorySeparator();
		path += foo;
		pckgs.push_back(path);
	}
	free(name_buf);
	free(index_buf);

	vector<CSLPackage_t> packages;

	// First read all headers. This is required to resolve the DEPENDENCIES
	for (const auto &packagePath : pckgs)
	{
		std::string packageFile(packagePath);
		packageFile += "/"; //XPLMGetDirectorySeparator();
		packageFile += "xsb_aircraft.txt";

		// Continue if file does not exist or package was already loaded
		if(!DoesFileExist(packageFile) || isPackageAlreadyLoaded(packagePath)) { continue; }

		XPLMDump() << XPMP_CLIENT_NAME ": Loading package: " << packageFile << "\n";
		std::string packageContent = GetFileContent(packageFile);
		auto package = ParsePackageHeader(packagePath, packageContent);
		if (package.hasValidHeader()) packages.push_back(package);
	}

	if (! packages.empty())
	{
		// iterator points to the first inserted package
		auto iterator = gPackages.insert(gPackages.end(), packages.begin(), packages.end());

		// Now we do a full run
		for (; iterator != gPackages.end(); ++iterator)
		{
			auto &package = *iterator;
			std::string packageFile(package.path);
			packageFile += "/"; //XPLMGetDirectorySeparator();
			packageFile += "xsb_aircraft.txt";
			std::string packageContent = GetFileContent(packageFile);
			ParseFullPackage(packageContent, package);
            // assign default values for vertOffset
            CSL_DefaultVertOffset(package);
		}
	}

#if 0
	::Microseconds((UnsignedWide*) &t2);
	double delta = (t2 - t1);
	delta /= 1000000.0;
	char buf[256];
	sprintf(buf,"CSL full load took: %lf\n", delta);
	XPLMDebugString(buf);
#endif	
	return ok;
}

/************************************************************************
 * CSL MATCHING
 ************************************************************************/

// Here's the basic idea: there are six levels of matching we can get,
// from the best (direct match of ICAO, airline and livery) to the worst
// (match an airplane's ICAO group but not ICAO, no livery or airline).
// So we will make six passes from best to worst, trying to match.  For
// each pass we try each package in turn from highest to lowest priority.


// These structs tell us how to build the matching keys for a given pass.
static	const int kUseICAO[] =		{ 1, 1, 0, 0, 1, 1, 0, 0};
static	const int kUseAirline[] =	{ 1, 1, 1, 1, 0, 0, 0, 0};
static	const int kUseLivery[] =	{ 1, 0, 1, 0, 1, 0, 1, 0};

CSLPlane_t *	CSL_MatchPlane(const char * inICAO, const char * inAirline, const char * inLivery, int * match_quality, bool use_default)
{
	XPLMPluginID	who;
	int		total, active;
	XPLMCountAircraft(&total, &active, &who);
	
	// First build up our various keys and info we need to do the match.
	string	icao(inICAO);
	string	airline(inAirline ? inAirline : "");
	string	livery(inLivery ? inLivery : "");
	string	group;
	string	key;

	map<string, string>::iterator group_iter = gGroupings.find(inICAO);
	if (group_iter != gGroupings.end())
		group = group_iter->second;

	char	buf[4096];

	if (gIntPrefsFunc("debug", "model_matching", 0))
	{
		sprintf(buf, XPMP_CLIENT_NAME " MATCH - ICAO=%s AIRLINE=%s LIVERY=%s GROUP=%s\n", icao.c_str(), airline.c_str(), livery.c_str(), group.c_str());
		XPLMDebugString(buf);
	}

	// Now we go through our six passes.
	for (int n = 0; n < match_count; ++n)
	{
		// Build up the right key for this pass.
		key = kUseICAO[n] ? icao : group;
		if (!kUseICAO[n] && group == "") {
			if (gIntPrefsFunc("debug", "model_matching", 0)) {
				sprintf(buf, XPMP_CLIENT_NAME " MATCH -    Skipping %d Due nil Group\n", n);
				XPLMDebugString(buf);
			}			
		}


		if (kUseLivery[n]) {
			if (livery == "") {
				if (gIntPrefsFunc("debug", "model_matching", 0)) {
					sprintf(buf, XPMP_CLIENT_NAME " MATCH -    Skipping %d Due Absent Livery\n", n);
					XPLMDebugString(buf);
				}
				continue;
			}
			key += " ";
			key += airline;
			key += " ";
			key += livery;
		}
		if (kUseAirline[n]) {
			if (airline == "") {
				if (gIntPrefsFunc("debug", "model_matching", 0)) {
					sprintf(buf, XPMP_CLIENT_NAME " MATCH -    Skipping %d Due Absent Airline\n", n);
					XPLMDebugString(buf);
				}
				continue;
			}
			key += " ";
			key += airline;
		}

		if (gIntPrefsFunc("debug", "model_matching", 0))
		{
			sprintf(buf, XPMP_CLIENT_NAME " MATCH -    Group %d key %s\n", n, key.c_str());
			XPLMDebugString(buf);
		}
		
		// Now go through each group and see if we match.
		for (size_t p = 0; p < gPackages.size(); ++p)
		{
			map<string,int>::iterator iter = gPackages[p].matches[n].find(key);
			if (iter != gPackages[p].matches[n].end())
				if (gPackages[p].planes[iter->second].plane_type != plane_Austin ||		// Special check - do NOT match a plane that isn't loaded.
						(gPackages[p].planes[iter->second].austin_idx != -1 && gPackages[p].planes[iter->second].austin_idx < total))
					if (gPackages[p].planes[iter->second].plane_type != plane_Obj ||
							gPackages[p].planes[iter->second].obj_idx != -1)
					{
						if (NULL != match_quality) *match_quality = n;

						if (gIntPrefsFunc("debug", "model_matching", 0)) {
							sprintf(buf, XPMP_CLIENT_NAME " MATCH - Found: %s/%s/%s : %s - %s\n", 
								gPackages[p].planes[iter->second].icao.c_str(),
								gPackages[p].planes[iter->second].airline.c_str(),
								gPackages[p].planes[iter->second].livery.c_str(),
								gPackages[p].planes[iter->second].file_path.c_str(),
								gPackages[p].planes[iter->second].texturePath.c_str());
							XPLMDebugString(buf);
						}

						return &gPackages[p].planes[iter->second];
					}
		}
	}

	if (gIntPrefsFunc("debug", "model_matching", 0))
	{
		XPLMDebugString(XPMP_CLIENT_NAME " MATCH - No match.\n");
	}
	if (NULL != match_quality) *match_quality = -1;



	// try the next step:
	// For each aircraft, we know the equiment type "L2T" and the WTC category.
	// try to find a model that has the same equipment type and WTC

	std::map<string, CSLAircraftCode_t>::const_iterator model_it = gAircraftCodes.find(icao);
	if(model_it != gAircraftCodes.end()) {

		if (gIntPrefsFunc("debug", "model_matching", 0))
		{
			XPLMDebugString(XPMP_CLIENT_NAME " MATCH/acf - Looking for a ");
			switch(model_it->second.category) {
			case 'L': XPLMDebugString(" light "); break;
			case 'M': XPLMDebugString(" medium "); break;
			case 'H': XPLMDebugString(" heavy "); break;
			default: XPLMDebugString(" funny "); break;
			}
			XPLMDebugString(model_it->second.equip.c_str());
			XPLMDebugString(" aircraft\n");
		}

		// 1. match WTC, full configuration ("L2P")
		// 2. match WTC, #engines and enginetype ("2P")
		// 3. match WTC, #egines ("2")
		// 4. match WTC, enginetype ("P")
		// 5. match WTC
		for(int megaPass = 1; megaPass <= 10; ++megaPass) {
            bool bMatchAirline = false;
            int pass = -1;
            switch(megaPass) {
                case 1:  bMatchAirline = true;  pass = 1; break;
                case 2:  bMatchAirline = true;  pass = 2; break;
                case 3:  bMatchAirline = false; pass = 1; break;
                case 4:  bMatchAirline = false; pass = 2; break;
                case 5:  bMatchAirline = true;  pass = 3; break;
                case 6:  bMatchAirline = true;  pass = 4; break;
                case 7:  bMatchAirline = false; pass = 3; break;
                case 8:  bMatchAirline = false; pass = 4; break;
                case 9:  bMatchAirline = true;  pass = 5; break;
                case 10: bMatchAirline = false; pass = 5; break;
            }
            
            // don't need the airline pass if we don't have one
            if (bMatchAirline && (!inAirline || !inAirline[0]))
                continue;
            
			if (gIntPrefsFunc("debug", "model_matching", 0))
			{
				switch(megaPass) {
                    case 1:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching airline, WTC and configuration\n");        break;
                    case 2:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching airline, WTC, #engines and enginetype\n"); break;
                    case 3:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching WTC and configuration\n");                 break;
                    case 4:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching WTC, #engines and enginetype\n");          break;
                    case 5:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching airline, WTC, #engines\n");                break;
                    case 6:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching airline, WTC, enginetype\n");              break;
                    case 7:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching WTC, #engines\n");                         break;
                    case 8:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching WTC, enginetype\n");                       break;
                    case 9:  XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching airline, WTC\n");                          break;
                    case 10: XPLMDebugString(XPMP_CLIENT_NAME " Match/acf - matching WTC\n");                                   break;
				}
			}

            for (CSLPackage_t& pckg: gPackages)
			{
                for (std::map<string, int>::const_iterator it = pckg.matches[bMatchAirline ? match_icao_airline : match_icao].cbegin();
                     it != pckg.matches[bMatchAirline ? match_icao_airline : match_icao].end();
                     ++it)
                {
                    CSLPlane_t& plane = pckg.planes[it->second];
					if (plane.plane_type != plane_Austin ||		// Special check - do NOT match a plane that isn't loaded.
						(plane.austin_idx != -1 && plane.austin_idx < total))
						if (plane.plane_type != plane_Obj || plane.obj_idx != -1)
						{
							// we have a candidate, lets see if it matches our criteria
                            const std::string icao (it->first.substr(0,4));
							std::map<string, CSLAircraftCode_t>::const_iterator m = gAircraftCodes.find(icao);
							if(m != gAircraftCodes.cend()) {
								// category
								bool match = (m->second.category == model_it->second.category);

								// make sure we have a valid equip type if we need it
								if(pass < 5 && m->second.equip.length() != 3) match = false;

								// engine type
								if(match && (pass <= 2 || pass == 4))
									match = (m->second.equip[2] == model_it->second.equip[2]);

								// #engines
								if(match && pass <= 3)
									match = (m->second.equip[1] == model_it->second.equip[1]);

								// full configuration string
								if(match && pass == 1)
									match = (m->second.equip == model_it->second.equip);
                                
                                // airline
                                if(match && bMatchAirline && (it->first.find(' ') != std::string::npos)) {
                                    const std::string airline (it->first.substr(it->first.find(' ')+1));
                                    match = (airline == inAirline);
                                }

								if(match) {
									// bingo
									if (gIntPrefsFunc("debug", "model_matching", 0))
									{
										XPLMDebugString(XPMP_CLIENT_NAME " MATCH/acf - found: ");
										XPLMDebugString(it->first.c_str());
										XPLMDebugString("\n");
									}
                                    return &plane;
								}
							}
						}
				}
			}
		}
	}

	if (gIntPrefsFunc("debug", "model_matching", 0)) {
		XPLMDebugString(string("gAircraftCodes.find(" + icao + ") returned no match.\n").c_str());
	}

	if (!strcmp(inICAO, gDefaultPlane.c_str())) return NULL;
	if (!use_default) return NULL;
	return CSL_MatchPlane(gDefaultPlane.c_str(), "", "", NULL, false);
}

void	CSL_Dump(void)
{
	// DIAGNOSTICS - print out everything we know.
	for (size_t n = 0; n < gPackages.size(); ++n)
	{
		XPLMDump() << XPMP_CLIENT_NAME " CSL: Package " << n << " path = " << gPackages[n].name << "\n";
		for (size_t p = 0; p < gPackages[n].planes.size(); ++p)
		{
			XPLMDump() << XPMP_CLIENT_NAME " CSL:         Plane " << p << " = " << gPackages[n].planes[p].file_path << "\n";
		}
		for (int t = 0; t < 6; ++t)
		{
			XPLMDump() << XPMP_CLIENT_NAME " CSL:           Table " << t << "\n";
			for (map<string, int>::iterator i = gPackages[n].matches[t].begin(); i != gPackages[n].matches[t].end(); ++i)
			{
				XPLMDump() << XPMP_CLIENT_NAME " CSL:                " << i->first << " -> " << i->second << "\n";
			}
		}
	}
}

/************************************************************************
 * CSL DRAWING
 ************************************************************************/

int				CSL_GetOGLIndex(CSLPlane_t *		model)
{
	switch(model->plane_type) {
	case plane_Austin:
		return model->austin_idx;
	case plane_Obj:
		if (model->texID != 0)
			return model->texID;
		return OBJ_GetModelTexID(model->obj_idx);
	default:
		return 0;
	}
}

// Plane drawing couldn't be simpler - it's just a "switch" between all
// of our drawing techniques.
void			CSL_DrawObject(
		XPMPPlanePtr			plane,
		float					distance,
		double 					x,
		double 					y,
		double 					z,
		double 					pitch,
		double 					roll,
		double 					heading,
		int						type,
		int	   					full,
		xpmp_LightStatus		lights,
		XPLMPlaneDrawState_t *	state)
{
	// Setup OpenGL for this plane render
	if(type != plane_Obj8)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(static_cast<GLfloat>(x), static_cast<GLfloat>(y), static_cast<GLfloat>(z));
		glRotatef(static_cast<GLfloat>(heading), 0.0, -1.0, 0.0);
		glRotatef(static_cast<GLfloat>(pitch), 01.0, 0.0, 0.0);
		glRotatef(static_cast<GLfloat>(roll), 0.0, 0.0, -1.0);
	}

	CSLPlane_t *model = plane->model;

	switch (type)
	{
	case plane_Austin:
	{
		XPLMPluginID	who;
		int		total, active;
		XPLMCountAircraft(&total, &active, &who);
		if (model->austin_idx > 0  && model->austin_idx < active)
			XPLMDrawAircraft(model->austin_idx,
							 static_cast<GLfloat>(x), static_cast<GLfloat>(y), static_cast<GLfloat>(z),
							 static_cast<GLfloat>(pitch), static_cast<GLfloat>(roll), static_cast<GLfloat>(heading),
							 full, state);
	}
		break;
	case plane_Obj:
		OBJ_PlotModel(plane, full ? distance : max(distance, 10000.0f),
					  x, y ,z, pitch, roll, heading);
		break;
	case plane_Lights:
		OBJ_DrawLights(plane, distance,
					   x, y ,z, pitch, roll, heading, lights);

		break;
	case plane_Obj8:
		obj_schedule_one_aircraft(
					model,
					x,
					y,
					z,
					pitch,
					roll,
					heading,
					full,		//
					lights,
					state);
		break;
	}

	if(type != plane_Obj8)
		glPopMatrix();
}
