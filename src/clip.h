#pragma once

#include <uv_stucco_intern.h>

typedef enum ClipCornerType {
	CLIP_ORIGIN_CLIP,
	CLIP_ORIGIN_SUBJECT,
	CLIP_INTERSECT,
	CLIP_ON_CLIP_EDGE,
	CLIP_ON_SUBJECT_EDGE,
	CLIP_ON_VERT
} ClipCornerType;

typedef struct ClipInfoOrigin {
	I32 corner;
} ClipInfoOrigin;

typedef struct ClipInfoIntersect {
	I32 clipCorner;
	I32 subjCorner;
	F32 clipAlpha;
	F32 subjAlpha;
} ClipInfoIntersect;

typedef struct ClipInfoOnEdge {
	I32 edgeCorner;
	I32 vertCorner;
	F32 alpha;
} ClipInfoOnEdge;

typedef struct ClipInfoOnVert {
	I32 clipCorner;
	I32 subjCorner;
} ClipInfoOnVert;

typedef union ClipInfo {
	ClipInfoOrigin origin;
	ClipInfoIntersect intersect;
	ClipInfoOnEdge onEdge;
	ClipInfoOnVert onVert;
} ClipInfo;

typedef struct ClipCorner {
	struct ClipCorner *pNext;
	ClipCornerType type;
	ClipInfo info;
} ClipCorner;

typedef struct ClipFaceRoot {
	ClipCorner *pRoot;
	I32 size;
} ClipFaceRoot;

typedef struct ClipFaceArr {
	LinAlloc cornerAlloc;
	ClipFaceRoot *pArr;
	I32 size;
	I32 count;
} ClipFaceArr;

Result stucClip(
	const StucAlloc *pAlloc,
	const void *pUserData,
	const Mesh *pClipMesh, const FaceRange *pClip,
	V3_F32 (* clipGetPos)(const void *, const Mesh *, const FaceRange *, I32),
	const Mesh *pSubjMesh, const FaceRange *pSubj,
	V3_F32 (* subjGetPos)(const void *, const Mesh *, const FaceRange *, I32),
	ClipFaceArr *pOut
);

void stucClipFaceArrDestroy(const StucAlloc *pAlloc, ClipFaceArr *pArr);