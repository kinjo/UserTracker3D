/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <stdio.h>
#include <GL/glew.h>
#include <math.h>
#include "SceneDrawer.h"

#ifndef USE_GLES
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#else
	#include "opengles.h"
#endif

extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

extern XnBool g_bDrawBackground;
extern XnBool g_bDrawPixels;
extern XnBool g_bDrawSkeleton;
extern XnBool g_bPrintID;
extern XnBool g_bPrintState;

extern XnBool g_bPrintFrameID;
extern XnBool g_bMarkJoints;

extern fd_set readfds;
extern int fd;
char wbuf[256];
extern struct timeval timeout;
extern bool is_connected;

#include <map>
std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	m_Errors[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& /*capability*/, const XnChar* /*strPose*/, XnUserID id, XnPoseDetectionStatus poseError, void* /*pCookie*/)
{
	m_Errors[id].second = poseError;
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}
#define SIZ 5.0f
#define VERTN 240
GLfloat vertices[VERTN * VERTN * 3];
GLfloat tex_coords[VERTN * VERTN * 2];
GLuint indices[VERTN * (VERTN - 1) * 2];
GLfloat texcoords[8];
GLuint buffer[2];
void initPlane(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	int i, j, k, l;
	GLfloat *v;

	// init vertices
	for (j = 0; j < VERTN; j++) {
		for (i = 0; i < VERTN; i++) {
			v = &vertices[(j * VERTN + i) * 3];
			v[0] = (float)topLeftX + (float)(bottomRightX - topLeftX) / (float)VERTN * (float)i
				+ (float)(bottomRightX - topLeftX) / 2.0f;
			v[1] = (float)topLeftY + (float)(bottomRightY - topLeftY) / (float)VERTN * (float)j
				+ (float)(bottomRightY - topLeftY) / 2.0f;
			v[2] = 0.0f;
		}
	}
	// init indices
	for (j = 0, k = 0; j < VERTN - 1; j++) {
		for (i = 0; i < VERTN; i++) {
			l = j % 2 == 0 ? i : VERTN - i - 1;
			indices[k++] = VERTN * j + l;
			indices[k++] = VERTN * (j + 1) + l;
		}
	}
	// init tex_coords
	for (j = 0; j < VERTN; j++) {
		for (i = 0; i < VERTN; i++) {
			v = &tex_coords[(j * VERTN + i) * 2];
			v[0] =  texcoords[0] - (texcoords[0] / (float)VERTN * (float)i);
			v[1] =  texcoords[1] - (texcoords[1] / (float)VERTN * (float)j);
		}
	}

	glGenBuffers(3, buffer);

	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer[1]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// tex_coords
	glBindBuffer(GL_ARRAY_BUFFER, buffer[2]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(tex_coords), tex_coords, GL_STATIC_DRAW);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	GLfloat verts[8] = {	topLeftX, topLeftY,
		topLeftX, bottomRightY,
		bottomRightX, bottomRightY,
		bottomRightX, topLeftY
	};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
	glFlush();
}
extern GLuint gl2Program;
extern GLfloat rotX, rotY;
extern GLfloat tranZ;
void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	static bool bInitialized = false;	
	if (!bInitialized) {
		initPlane(topLeftX, topLeftY, bottomRightX, bottomRightY);
		bInitialized = true;
	}

	glBindBuffer(GL_ARRAY_BUFFER, buffer[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glUseProgram(gl2Program);

	glPushMatrix();
	{
		glRotatef(rotY, 0.0, 1.0, 0.0);
		glRotatef(rotX, 1.0, 0.0, 0.0);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		glBindBuffer(GL_ARRAY_BUFFER, buffer[0]);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, buffer[2]);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer[1]);
		glDrawElements(GL_TRIANGLE_STRIP, sizeof(indices)/sizeof(GLuint), GL_UNSIGNED_INT, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
	}
	glPopMatrix();
	glUseProgram(0);

	glDisable(GL_DEPTH_TEST);
	glFlush();
}

XnFloat Colors[][3] =
{
	{0,1,1},
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{1,1,1}
};
XnUInt32 nColors = 10;
#ifndef USE_GLES
void glPrintString(void *font, char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}
#endif

float norm(float v[]) {
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float innerProduct(float v1[], float v2[]) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

float vectorAngle(float v1[], float v2[])
{
	return acos(innerProduct(v1, v2) / (norm(v1) * norm(v2)));
}

void outerProduct(float v0[], float v1[], float v2[])
{
	v0[0] = v1[1] * v2[2] - v1[2] * v2[1]; 
	v0[1] = v1[2] * v2[0] - v1[0] * v2[2]; 
	v0[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void mulVector(float v0[], float v1[], float v2[]) {
	v0[0] = v1[0] * v2[0];
	v0[1] = v1[1] * v2[1];
	v0[2] = v1[2] * v2[2];
}

float toDegree(float rad) {
	return rad * 180.0 / (atan(1.0) * 4.0);
}

#define CMD_UPDATEVEC 1
bool DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return true;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint1) ||
		!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint2))
	{
		return false;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return true;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	if (eJoint2 == XN_SKEL_RIGHT_HAND) {
		if (is_connected) {
			XnSkeletonJointPosition head_joint_pos;
			g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, XN_SKEL_HEAD, head_joint_pos);
			XnPoint3D hand_pos = head_joint_pos.position;

			XnSkeletonJointPosition left_elbow_joint_pos;
			g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player,
				XN_SKEL_LEFT_ELBOW,
				left_elbow_joint_pos);
			XnPoint3D left_elbow_pos = left_elbow_joint_pos.position;

			XnSkeletonJointPosition left_hand_joint_pos;
			g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player,
				XN_SKEL_LEFT_HAND,
				left_hand_joint_pos);
			XnPoint3D left_hand_pos = left_hand_joint_pos.position;

			FD_ZERO(&readfds);
			FD_SET(fd, &readfds);
			select(32, &readfds, NULL, NULL, &timeout);

			float v[16];
			int len, i;
			if (FD_ISSET(fd, &readfds)) {
				for (i = 0; i < sizeof(v); i += len) {
					len = read(fd, (char *)v + i, sizeof(v) - i);
					if (len == -1) {
						perror("read error");
						close(fd);
						fd = 0;
					}
				}

				// pt[1] is a coordinate for the right hand
				v[0] = pt[1].X;
				v[1] = pt[1].Y;
				v[2] = pt[1].Z;
				v[3] = 1.0f;

				// get a vector to right hand from head
				{
					// head -> right hand vector
					float vec0[] = {
						pt[1].X - hand_pos.X,
						pt[1].Y - hand_pos.Y,
						pt[1].Z - hand_pos.Z
					};
					// base vector
					float vec1[] = {0, 0, -1};
					float f[] = {1, 0, 1}, v1[3], v2[3];

					mulVector(v1, vec0, f);
					mulVector(v2, vec1, f);

					float direction = vectorAngle(v1, v2);
					direction *= vec0[0] < 0 ? -1 : 1;
					direction = direction < 0 ? 2 * M_PI + direction : direction;

					// get direction and set to sending vector
					v[4] = direction;
					v[5] = 0;
					v[6] = 0;
					v[7] = 0;
				}

				// set left hand coordinate (unused)
				{
					v[8] = left_hand_pos.X;
					v[9] = left_hand_pos.Y;
					v[10] = left_hand_pos.Z;
					v[11] = 1.0f;
				}

				// unused
				{
					v[12] = 0.0f;
					v[13] = 0.0f;
					v[14] = 0.0f;
					v[15] = 0.0f;
				}

				// ntoh
				for (i = 0; i < sizeof(v) / sizeof(float); i++) 
					*(uint32_t *)&v[i] = ntohl(*((uint32_t *)v + i));

				// set a command to update vector in distribution server
				i = CMD_UPDATEVEC;
				i = htonl(i);

				// set buffer
				memcpy(wbuf, &i, sizeof(i));
				memcpy(wbuf + sizeof(i), v, sizeof(v));

				// write
				if (write(fd, wbuf, sizeof(i) + sizeof(v)) == -1) {
					perror("write error");
					close(fd);
					fd = 0;
				}
			}
		}
	}

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
#define ADJUST_LIMB_Z_RATIO 2.0f
#define ADJUST_LIMB_Z 400.0f
#define STATIC_DMD_W 640
#define STATIC_DMD_H 480
#ifndef USE_GLES
	glVertex3i(pt[0].X - STATIC_DMD_W / 2, pt[0].Y - STATIC_DMD_H / 2, pt[0].Z / ADJUST_LIMB_Z_RATIO - ADJUST_LIMB_Z);
	glVertex3i(pt[1].X - STATIC_DMD_W / 2, pt[1].Y - STATIC_DMD_H / 2, pt[1].Z / ADJUST_LIMB_Z_RATIO - ADJUST_LIMB_Z);
#else
	GLfloat verts[4] = {pt[0].X, pt[0].Y, pt[1].X, pt[1].Y};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINES, 0, 2);
	glFlush();
#endif

	return true;
}

static const float DEG2RAD = 3.14159/180;
 
void drawCircle(float x, float y, float radius)
{
   glBegin(GL_TRIANGLE_FAN);
 
   for (int i=0; i < 360; i++)
   {
      float degInRad = i*DEG2RAD;
      glVertex2f(x + cos(degInRad)*radius, y + sin(degInRad)*radius);
   }
 
   glEnd();
}
void DrawJoint(XnUserID player, XnSkeletonJoint eJoint)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint))
	{
		return;
	}

	XnSkeletonJointPosition joint;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt;
	pt = joint.position;

	g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

	drawCircle(pt.X, pt.Y, 2);
}

const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
{
	switch (error)
	{
	case XN_CALIBRATION_STATUS_OK:
		return "OK";
	case XN_CALIBRATION_STATUS_NO_USER:
		return "NoUser";
	case XN_CALIBRATION_STATUS_ARM:
		return "Arm";
	case XN_CALIBRATION_STATUS_LEG:
		return "Leg";
	case XN_CALIBRATION_STATUS_HEAD:
		return "Head";
	case XN_CALIBRATION_STATUS_TORSO:
		return "Torso";
	case XN_CALIBRATION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_CALIBRATION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_CALIBRATION_STATUS_POSE:
		return "Pose";
	default:
		return "Unknown";
	}
}
const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
{
	switch (error)
	{
	case XN_POSE_DETECTION_STATUS_OK:
		return "OK";
	case XN_POSE_DETECTION_STATUS_NO_USER:
		return "NoUser";
	case XN_POSE_DETECTION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_POSE_DETECTION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_POSE_DETECTION_STATUS_ERROR:
		return "General error";
	default:
		return "Unknown";
	}
}

void subVector(float v0[], float v1[], float v2[])
{
	v0[0] = v1[0] - v2[0];
	v0[1] = v1[1] - v2[1];
	v0[2] = v1[2] - v2[2];
}

void jointToVector(float v0[], XnSkeletonJointPosition joint)
{
	v0[0] = joint.position.X;
	v0[1] = joint.position.Y;
	v0[2] = joint.position.Z;
}

void twoJointToVector(float v0[], XnUserID *user, XnSkeletonJoint j0, XnSkeletonJoint j1)
{
	float v1[3], v2[3];
	XnSkeletonJointPosition p0, p1;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(*user, j0, p0);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(*user, j1, p1);
	jointToVector(v1, p0);
	jointToVector(v2, p1);
	subVector(v0, v1, v2);
}

float fourJointsAngle(XnUserID *user, XnSkeletonJoint j0, XnSkeletonJoint j1, XnSkeletonJoint j2, XnSkeletonJoint j3)
{
	float v1[3], v2[3];
	twoJointToVector(v1, user, j0, j1);
	twoJointToVector(v2, user, j2, j3);
	return vectorAngle(v1, v2);
}

void fourJointsOuterProduct(float v0[], XnUserID *user, XnSkeletonJoint j0, XnSkeletonJoint j1, XnSkeletonJoint j2, XnSkeletonJoint j3)
{
	float v1[3], v2[3];
	twoJointToVector(v1, user, j0, j1);
	twoJointToVector(v2, user, j2, j3);
	outerProduct(v0, v2, v1);
}

extern GLfloat rotX, rotY;

#define ADJUST_PLANE_DEPTH 2.0f
void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd)
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());

		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;
	}

	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();

	static unsigned int nZRes = dmd.ZRes();
	static float* pDepthHist = (float*)malloc(nZRes* sizeof(float));

	// Calculate the accumulative histogram
	memset(pDepthHist, 0, nZRes*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			GLfloat *v;
			v = &vertices[
				((int)((float)(g_nYRes - nY - 1)/ (float)g_nYRes * (float)VERTN) * VERTN +
					(int)((float)(g_nXRes - nX - 1) / (float)g_nXRes * (float)VERTN)) * 3];
			v[2] = (float)nValue / ADJUST_PLANE_DEPTH;

			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<nZRes; nIndex++)
	{
		pDepthHist[nIndex] += pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<nZRes; nIndex++)
		{
			pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dmd.Data();
	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = pDepthHist[nValue];

						pDestImage[0] = nHistValue * Colors[nColorID][0]; 
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}

	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dmd.XRes(),dmd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);

	glPushMatrix();
	{
		glRotatef(rotY, 0.0, 1.0, 0.0);
		glRotatef(rotX, 1.0, 0.0, 0.0);

		char strLabel[50] = "";
		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
		{
#ifndef USE_GLES
			if (g_bPrintID)
			{
				XnPoint3D com;
				g_UserGenerator.GetCoM(aUsers[i], com);
				g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);

				XnUInt32 nDummy = 0;

				xnOSMemSet(strLabel, 0, sizeof(strLabel));
				if (!g_bPrintState)
				{
					// Tracking
					xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", aUsers[i]);
				}
				else if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
				{
					// Tracking
					xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", aUsers[i]);
				}
				else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
				{
					// Calibrating
					xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", aUsers[i], GetCalibrationErrorString(m_Errors[aUsers[i]].first));
				}
				else
				{
					// Nothing
					xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Looking for pose [%s]", aUsers[i], GetPoseErrorString(m_Errors[aUsers[i]].second));
				}


				glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

				glRasterPos2i(com.X, com.Y);
				glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
			}
#endif
			if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);

				// Draw Joints
				if (g_bMarkJoints)
				{
					// Try to draw all joints
					DrawJoint(aUsers[i], XN_SKEL_HEAD);
					DrawJoint(aUsers[i], XN_SKEL_NECK);
					DrawJoint(aUsers[i], XN_SKEL_TORSO);
					DrawJoint(aUsers[i], XN_SKEL_WAIST);

					DrawJoint(aUsers[i], XN_SKEL_LEFT_COLLAR);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_SHOULDER);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_ELBOW);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_WRIST);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_HAND);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_FINGERTIP);

					DrawJoint(aUsers[i], XN_SKEL_RIGHT_COLLAR);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_SHOULDER);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_ELBOW);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_WRIST);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_HAND);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_FINGERTIP);

					DrawJoint(aUsers[i], XN_SKEL_LEFT_HIP);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_KNEE);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_ANKLE);
					DrawJoint(aUsers[i], XN_SKEL_LEFT_FOOT);

					DrawJoint(aUsers[i], XN_SKEL_RIGHT_HIP);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_KNEE);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_ANKLE);
					DrawJoint(aUsers[i], XN_SKEL_RIGHT_FOOT);
				}

#ifndef USE_GLES
				glBegin(GL_LINES);
#endif

				{
					float d0, d1, d2, d3;
					XnSkeletonJointPosition j0, j1;

					d0 = fourJointsAngle(&aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW,
								XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_ELBOW);
					d1 = fourJointsAngle(&aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_RIGHT_SHOULDER,
								XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_SHOULDER);

					d2 = fourJointsAngle(&aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW,
								XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_ELBOW);
					d3 = fourJointsAngle(&aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_LEFT_SHOULDER,
								XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_SHOULDER);

					if (d0 > 2.2 && d1 > 2.7) {
						if (d2 > 2.2 && d3 > 2.7) {
							tranZ += 8.3f;
						} else {
							rotY += 1.3f;
							if (rotY > 360.0f) rotY = 0.0f;
							if (rotY < 0.0f) rotY = 360.0f;
						}
					} else if (d2 > 2.2 && d3 > 2.7) {
						rotY -= 1.3f;
						if (rotY > 360.0f) rotY = 0.0f;
						if (rotY < 0.0f) rotY = 360.0f;
					}

					if (d0 > 2.2 && d1 < 2.4 && d2 > 2.2 && d3 < 2.4) {
						float q0[3], q1[3], q2[3];
						fourJointsOuterProduct(q0, &aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_LEFT_SHOULDER,
							XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_SHOULDER);
						twoJointToVector(q1, &aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_HAND);
						twoJointToVector(q2, &aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_HAND);
						float e0 = vectorAngle(q0, q1);
						float e1 = vectorAngle(q1, q2);
						if (e0 < 0.6 && e1 < 0.6) {
							tranZ -= 8.3f;
						}
					}

				}

				// Draw Limbs
				DrawLimb(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);

				DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
				DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
				if (!DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST))
				{
					DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
				}
				else
				{
					DrawLimb(aUsers[i], XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND);
					DrawLimb(aUsers[i], XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP);
				}


				DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
				if (!DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_WRIST))
				{
					DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
				}
				else
				{
					DrawLimb(aUsers[i], XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND);
					DrawLimb(aUsers[i], XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP);
				}

				DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

				DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
				DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
				DrawLimb(aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

				DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

				DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
#ifndef USE_GLES
				glEnd();
#endif
			}
		}
	}
	glPopMatrix();

	if (g_bPrintFrameID)
	{
		static XnChar strFrameID[80];
		xnOSMemSet(strFrameID, 0, 80);
		XnUInt32 nDummy = 0;
		xnOSStrFormat(strFrameID, sizeof(strFrameID), &nDummy, "%d", dmd.FrameID());

		glColor4f(1, 0, 0, 1);

		glRasterPos2i(10, 10);

		glPrintString(GLUT_BITMAP_HELVETICA_18, strFrameID);
	}
}
