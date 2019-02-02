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
 
#include "XPMPMultiplayerCSLOffset.h"
#include "XPMPMultiplayerVars.h"
#include "XPLMUtilities.h"
#include <stdio.h>
#include <algorithm>
#include <errno.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <functional>
#include <cctype>
#include <vector>

CslModelVertOffsetCalculator cslVertOffsetCalc;

CslModelVertOffsetCalculator::~CslModelVertOffsetCalculator() {
	saveUserOffsets();
}

void CslModelVertOffsetCalculator::setResourcesDir(const string &inResDir) {
	if (mResourcesDir != inResDir) {
		mResourcesDir = inResDir;
		loadUserOffsets();
	}
}

void CslModelVertOffsetCalculator::findOrUpdateActualVertOffset(CSLPlane_t & inOutCslModel) {
	// plane updating user vert offset if it's needed
	if (!mUpdateUserOffsetForThisMtl.empty()) {
		// check if we want to update user offset for this mtl
		auto result = mUpdateUserOffsetForThisMtl.find(inOutCslModel.getModelName());
		if (result != mUpdateUserOffsetForThisMtl.end()) {
			inOutCslModel.isUserVertOffsetUpToDate = false;
			mUpdateUserOffsetForThisMtl.erase(result);
		}
	}
	// set/update user vert offset if it's needed
	if (!inOutCslModel.isUserVertOffsetUpToDate) {
		inOutCslModel.isUserVertOffsetUpToDate = true;
		inOutCslModel.userVertOffset = 0.0;
		inOutCslModel.isUserVertOffsetAvail = false;
		// search for specific user offset
		auto result2 = mAvailableUserOffsets.find(inOutCslModel.getModelName());
		if (result2 != mAvailableUserOffsets.end()) {
			inOutCslModel.userVertOffset = result2->second;
			inOutCslModel.isUserVertOffsetAvail = true;
			XPLMDebugString(std::string(XPMP_CLIENT_NAME ": The USER Y offset (" + std::to_string(inOutCslModel.userVertOffset) 
										+ ") for the model has been found; Mtl Code: " + inOutCslModel.getModelName() + "\n").c_str());
		}
	}
	
	// just check and log the fact of avail of xsb offset
	if (!inOutCslModel.isXsbVertOffsetUpToDate) {
		inOutCslModel.isXsbVertOffsetUpToDate = true;
		if(inOutCslModel.isXsbVertOffsetAvail) {
			XPLMDebugString(std::string(XPMP_CLIENT_NAME ": The Y offset (" + std::to_string(inOutCslModel.xsbVertOffset) 
										+ ") for the model has been found in the xsb file; Mtl Code: " + inOutCslModel.getModelName() + "\n").c_str());
		}
	}
	
	// set/update calculated vert offset if it's needed
	if (!inOutCslModel.isCalcVertOffsetUpToDate) {
		inOutCslModel.isCalcVertOffsetUpToDate = true;
		inOutCslModel.calcVertOffset = 0.0;
		inOutCslModel.isCalcVertOffsetAvail = false;
		if (inOutCslModel.plane_type == plane_Obj) {
			//trying to calculate it from the obj file
			findOffsetInObj(inOutCslModel);
		}
		else if (inOutCslModel.plane_type == plane_Obj8) {
			//trying to calculate it from the obj8 file
			findOffsetInObj8(inOutCslModel);
		}
	}
	
	//-------------------------------------------------------------------
	// setup actual Y offset
	inOutCslModel.actualVertOffsetType = eVertOffsetType::none;
	// if user offset is avail
	if (inOutCslModel.isUserVertOffsetAvail) {
		inOutCslModel.actualVertOffset = inOutCslModel.userVertOffset;
		inOutCslModel.actualVertOffsetType = eVertOffsetType::user;
	}
	// if xsb offset is avail
	else if (inOutCslModel.isXsbVertOffsetAvail) {
		inOutCslModel.actualVertOffset = inOutCslModel.xsbVertOffset;
		inOutCslModel.actualVertOffsetType = eVertOffsetType::xsb;
	}
	// if calc offset is avail
	else if (inOutCslModel.isCalcVertOffsetAvail) {
		inOutCslModel.actualVertOffset = inOutCslModel.calcVertOffset;
		inOutCslModel.actualVertOffsetType = eVertOffsetType::calculated;
	}
	// if something wrong and we not have any vert offsets, use 0.0
	if (inOutCslModel.actualVertOffsetType == eVertOffsetType::none) {
		XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found."
				" Will use 0 as the vert offset. Mtl code: " + inOutCslModel.getModelName() + "\n").c_str());
		inOutCslModel.calcVertOffset = 0.0;
		inOutCslModel.isCalcVertOffsetAvail = true;
		inOutCslModel.actualVertOffset = inOutCslModel.calcVertOffset;
		inOutCslModel.actualVertOffsetType = eVertOffsetType::default_offset;
	}
	if (inOutCslModel.actualVertOffsetType != inOutCslModel.prevActualVertOffsetType) {
		inOutCslModel.prevActualVertOffsetType = inOutCslModel.actualVertOffsetType;
		XPLMDebugString(std::string(XPMP_CLIENT_NAME ": Using the " + offsetTypeToString(inOutCslModel.actualVertOffsetType) 
				+ " Y offset (" + std::to_string(inOutCslModel.actualVertOffset) + ") for the model. Mtl code: " 
				+ inOutCslModel.getModelName() + "\n").c_str());
	}
}

void CslModelVertOffsetCalculator::actualVertOffsetInfo(const std::string &inMtl, std::string &outType, double &outOffset) {
	for (auto &package : gPackages) {
		for (auto &model : package.planes) {
			if (model.getModelName() == inMtl) {
				findOrUpdateActualVertOffset(model);
				outType = offsetTypeToString(model.actualVertOffsetType);
				outOffset = model.actualVertOffset;
				return;
			}
		}
	}
}

void CslModelVertOffsetCalculator::setUserVertOffset(const string &inMtlCode, double inOffset) {
	mAvailableUserOffsets[inMtlCode] = inOffset;
	mUpdateUserOffsetForThisMtl.emplace(inMtlCode);
}

void CslModelVertOffsetCalculator::removeUserVertOffset(const string &inMtlCode) {
	mAvailableUserOffsets.erase(inMtlCode);
	mUpdateUserOffsetForThisMtl.emplace(inMtlCode);
}

/**************************************************************************************************/
///////////////////////////////////////////* Functions *////////////////////////////////////////////
/**************************************************************************************************/

bool CslModelVertOffsetCalculator::findOffsetInObj8(CSLPlane_t &inOutCslModel) {
	// todo: add warning for rotate and translate
	if (inOutCslModel.plane_type == plane_Obj8) {
		double min = 0.0;
		double max = 0.0;
		bool isRotateOrTranslateAnimDetected = false;
		for (auto &obj8 : inOutCslModel.attachments) {
			if (obj8.draw_type == draw_solid) {
				std::ifstream file(obj8.file.c_str(), std::ios_base::in);
				if (!file.is_open()) {
					XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj8. Can't open the file: " + obj8.file + "\n").c_str());
					return false;
				}
				int coordLinesNumber = 0;
				for (size_t lineNumber = 0; file.good(); lineNumber++) {
					std::string line;
					std::getline(file, line);
					line = xmp::trim(line);
					if (line.size() == 0 || line.at(0) == ';' || line.at(0) == '#' || line.at(0) == '/') continue;
					if (lineNumber == 1 && atoi(line.c_str()) < 800) {
						XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj. "
							"Expect obj8 or higher but obj7 or lower is given. The obj: " + obj8.file + "\n").c_str());
						return false;
					}
					std::vector<std::string> tokens;
					tokens = xmp::explode(line);
					if (tokens.size() >= 4) {
						if (xmp::trim(tokens[0]) == "VT" || xmp::trim(tokens[0]) == "VLINE") {
							double y = std::atof(xmp::trim(tokens[2]).c_str());
							if (y < min) {
								min = y;
							}
							if (y > max) {
								max = y;
							}
							++coordLinesNumber;
						}
					}
					if (xmp::trim(tokens[0]) == "ANIM_trans" || xmp::trim(tokens[0]) == "ANIM_rotate") {
						isRotateOrTranslateAnimDetected = true;
					}
				}
				if (coordLinesNumber < 3) {
					XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj8. "
						"Number of lines with coordinates (VT&VLINE) is too small. The obj: " + obj8.file + "\n").c_str());
					return false;
				}
			}
		}
		if (min < 0.0) {
			inOutCslModel.calcVertOffset = std::fabs(min);
		}
		else {
			inOutCslModel.calcVertOffset = -max;
		}
		inOutCslModel.isCalcVertOffsetAvail = true;
		if (isRotateOrTranslateAnimDetected) {
			XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: During calculating the Y offset for the model Translate or/and Rotate animation has been found in an obj8; "
				"So, the calculated Y offset can be wrong due to animations. "
				"Mtl code: " + inOutCslModel.getModelName() + "\n").c_str());
		}
		XPLMDebugString(std::string(XPMP_CLIENT_NAME ": The Y offset (" + std::to_string(inOutCslModel.calcVertOffset) + ") for the model has been calculated from the obj8; "
			"Mtl code: " + inOutCslModel.getModelName() + "\n").c_str());
		return true;
	}
	return false;
}

bool CslModelVertOffsetCalculator::findOffsetInObj(CSLPlane_t &inOutCslModel) {
	if (inOutCslModel.plane_type == plane_Obj) {
		std::ifstream file(inOutCslModel.file_path.c_str(), std::ios_base::in);
		if (!file.is_open()) {
			XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj. Can't open the file: " + inOutCslModel.file_path + "\n").c_str());
			return false;
		}
		double min = 0.0;
		double max = 0.0;
		int coordLinesNumber = 0;
		for (size_t lineNumber = 0; file.good(); lineNumber++) {
			std::string line;
			std::getline(file, line);
			line = xmp::trim(line);
			if (line.size() == 0 || line.at(0) == ';' || line.at(0) == '#' || line.at(0) == '/') continue;
			if (lineNumber == 1 && atoi(line.c_str()) >= 800) {
				XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj. "
					"Expect obj6 or obj7 but obj8 or higher is given. The obj: " + inOutCslModel.file_path + "\n").c_str());
				return false;
			}
			std::vector<std::string> tokens;
			tokens = xmp::explode(line);
			if (tokens.size() >= 3) {
				double x, y, z;
				x = std::atof(tokens[0].c_str());
				y = std::atof(tokens[1].c_str());
				z = std::atof(tokens[2].c_str());
				if (x != 0.0 && y != 0.0 && z != 0.0) {
					if (y < min) {
						min = y;
					}
					if (y > max) {
						max = y;
					}
					++coordLinesNumber;
				}
			}
		}
		if (coordLinesNumber < 3) {
			XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: The Y offset for the model is not found in obj. "
				"Number of lines with coordinates is too small. The obj: " + inOutCslModel.file_path + "\n").c_str());
			return false;
		}
		if (min < 0.0) {
			inOutCslModel.calcVertOffset = std::fabs(min);
		}
		else {
			inOutCslModel.calcVertOffset = -max;
		}
		inOutCslModel.isCalcVertOffsetAvail = true;
		XPLMDebugString(std::string(XPMP_CLIENT_NAME ": The Y offset (" + std::to_string(inOutCslModel.calcVertOffset) + ") for the model has been calculated from its obj files; "
			"Mtl code: " + inOutCslModel.getModelName() + "\n").c_str());
		return true;
	}
	return false;
}

void CslModelVertOffsetCalculator::loadUserOffsets() {
	std::string fileName = mResourcesDir + "userVertOffsets.txt";
	std::ifstream file(fileName.c_str(), std::ios_base::in);
	if (!file.is_open()) {
		XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: Can't open the user vertical offsets file: " + fileName + "\n").c_str());
		return;
	}
	mAvailableUserOffsets.clear();
	while (file.good()) {
		std::string line;
		std::getline(file, line);
		line = xmp::trim(line);
		if (line.size() == 0 || line.at(0) == ';' || line.at(0) == '#' || line.at(0) == '/') continue;
		std::vector<std::string> tokens;
		tokens = xmp::explode(line, ",");
		if (tokens.size() < 2) continue;
		if (tokens[0].size() <= 4) {// only icao
			mAvailableUserOffsets.emplace(xmp::trim(tokens[0]), std::atof(tokens[1].c_str()));
		}
		else if (tokens[0].size() == 7) {// icao and airline
			mAvailableUserOffsets.emplace(xmp::trim(tokens[0].substr(0, 4)) + xmp::trim(tokens[0].substr(4)),
				std::atof(tokens[1].c_str()));
		}
		else if (tokens[0].size() > 7) {// icao, airline, livery
			mAvailableUserOffsets.emplace(xmp::trim(tokens[0].substr(0, 4))
				+ xmp::trim(tokens[0].substr(4, 3))
				+ xmp::trim(tokens[0].substr(7)),
				std::atof(tokens[1].c_str()));
		}
		else {
			continue;
		}
	}
}

void CslModelVertOffsetCalculator::saveUserOffsets() {
	std::string fileName = mResourcesDir + "userVertOffsets.txt";
	std::ofstream file(fileName.c_str(), std::ios_base::out);
	if (!file.is_open()) {
		XPLMDebugString(std::string(XPMP_CLIENT_NAME " Warning: Can't open the user vertical offsets file: " + fileName + "\n").c_str());
		return;
	}
	for (auto &item : mAvailableUserOffsets) {
		file << item.first << ", " << item.second << std::endl;
	}
	file.close();
}

std::string CslModelVertOffsetCalculator::offsetTypeToString(eVertOffsetType inOffsetType) {
	switch (inOffsetType) {
	case eVertOffsetType::default_offset:
		return "Default";
		break;
	case eVertOffsetType::user:
		return "User";
		break;
	case eVertOffsetType::xsb:
		return "Xsb";
		break;
	case eVertOffsetType::calculated:
		return "Calculated";
		break;
	case eVertOffsetType::none:
	default:
		return "Not defined";
		break;
	}
}
