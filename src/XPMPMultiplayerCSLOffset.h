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

#ifndef XPLMMULTIPLAYERCSLOFFSET_H
#define XPLMMULTIPLAYERCSLOFFSET_H

#include "XPLMPlanes.h"
#include "XPMPMultiplayerVars.h"

class CslModelVertOffsetCalculator {

public:
	~CslModelVertOffsetCalculator();
	
	static std::string offsetTypeToString(eVertOffsetType inOffsetType);
	
	void setResourcesDir(const std::string &inResDir);
	void findOrUpdateActualVertOffset(CSLPlane_t &inOutCslModel);
	
	void actualVertOffsetInfo(const std::string &inMtl, std::string &outType, double &outOffset);
	void setUserVertOffset(const std::string &inMtlCode, double inOffset);
	void removeUserVertOffset(const std::string &inMtlCode);
	
private:
	bool findOffsetInObj8(CSLPlane_t &inOutCslModel);
	bool findOffsetInObj(CSLPlane_t &inOutCslModel);
	
	void loadUserOffsets();
	void saveUserOffsets();
	
	std::string mResourcesDir;
	std::map<std::string, double>mAvailableUserOffsets;
	std::set<std::string> mUpdateUserOffsetForThisMtl;
};

extern CslModelVertOffsetCalculator cslVertOffsetCalc;

#endif
