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

typedef struct ClipCornerIdx {
	I32 boundary;
	I32 corner;
} ClipCornerIdx;

typedef struct ClipInfoOrigin {
	ClipCornerIdx corner;
} ClipInfoOrigin;

typedef struct ClipInfoIntersect {
	ClipCornerIdx clipCorner;
	ClipCornerIdx subjCorner;
	F32 clipAlpha;
	F32 subjAlpha;
} ClipInfoIntersect;

typedef struct ClipInfoOnEdge {
	ClipCornerIdx edgeCorner;
	ClipCornerIdx vertCorner;
	F32 alpha;
} ClipInfoOnEdge;

typedef struct ClipInfoOnVert {
	ClipCornerIdx clipCorner;
	ClipCornerIdx subjCorner;
} ClipInfoOnVert;

typedef union ClipInfo {
	ClipInfoOrigin origin;
	ClipInfoIntersect intersect;
	ClipInfoOnEdge onEdge;
	ClipInfoOnVert onVert;
} ClipInfo;

typedef struct ClipCorner {
	struct ClipCorner *pNext;
	struct ClipCorner *pPrev;
	V3_F32 pos;
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

typedef struct ClipInput {
	I32 *pSizes;
	I32 boundaries;
	const void *pUserData;
} ClipInput;

struct Corner;

typedef struct FaceRoot {
	struct Corner *pRoot;
	I32 size;
	I32 originSize;
	I32 boundary;
} FaceRoot;

typedef struct ClipFaceIntern {
	FaceRoot *pRoots;
	I32 boundaries;
} ClipFaceIntern;

typedef enum ClipOrSubj {
	FACE_CLIP,
	FACE_SUBJECT
} ClipOrSubj;

stucClipIntern(
	const StucAlloc *pAlloc,
	LinAlloc *pCornerAlloc,
	I32 initSize,
	ClipFaceIntern *pClip, ClipFaceIntern *pSubj,
	ClipFaceArr *pOut
);

stucClipInitMem(
	const StucAlloc *pAlloc,
	ClipInput clipInput, ClipInput subjInput,
	LinAlloc *pRootAlloc,
	LinAlloc *pCornerAlloc,
	I32 *pInitSize
);

stucClipInitCorner(
	ClipFaceIntern *pFace,
	I32 boundary,
	I32 corner,
	ClipOrSubj face,
	V3_F32 pos
);

typedef struct ClipFuncs {
	V2_F32 (* getClipPos)(const void *, const void *, ClipInput, I32, I32);
	V3_F32 (* getSubjPos)(const void *, const void *, ClipInput, I32, I32);
} ClipFuncs;

STUC_FORCE_INLINE
V3_F32 callGetClipPos(
	const void *pUserData,
	const void *pMesh,
	const ClipFuncs *pFuncs,
	ClipInput inputFace,
	I32 boundary,
	I32 corner
) {
	V2_F32 pos = pFuncs->getClipPos(pUserData, pMesh, inputFace, boundary, corner);
	return (V3_F32) {.d = {pos.d[0], pos.d[1], .0f}};
}

STUC_FORCE_INLINE
V3_F32 callGetSubjPos(
	const void *pUserData,
	const void *pMesh,
	const ClipFuncs *pFuncs,
	ClipInput inputFace,
	I32 boundary,
	I32 corner
) {
	return pFuncs->getSubjPos(pUserData, pMesh, inputFace, boundary, corner);
}

STUC_FORCE_INLINE
void cornerListInit(
	LinAlloc *pRootAlloc,
	LinAlloc *pCornerAlloc,
	const void *pUserData,
	const void *pMesh, ClipInput inputFace,
	V3_F32 (* getPos)(const void *, const void *, const ClipFuncs *, ClipInput, I32, I32),
	const ClipFuncs *pFuncs,
	ClipFaceIntern *pFace,
	ClipOrSubj face
) {
	pFace->boundaries = inputFace.boundaries;
	stucLinAlloc(pRootAlloc, &pFace->pRoots, pFace->boundaries);
	for (I32 i = 0; i < inputFace.boundaries; ++i) {
		pFace->pRoots[i].size = inputFace.pSizes[i];
		pFace->pRoots[i].originSize = pFace->pRoots[i].size;
		pFace->pRoots[i].boundary = i;
		stucLinAlloc(pCornerAlloc, &pFace->pRoots[i].pRoot, inputFace.pSizes[i]);
		for (I32 j = 0; j < inputFace.pSizes[i]; ++j) {
			V3_F32 pos = getPos(pUserData, pMesh, pFuncs, inputFace, i, j);
			stucClipInitCorner(pFace, i, j, face, pos);
		}
	}
}

static inline
Result stucClip(
	const StucAlloc *pAlloc,
	const void *pUserData,
	const void *pClipMesh, ClipInput clipInput,
	V2_F32 (* clipGetPos)(const void *, const void *, ClipInput, I32, I32),
	const void *pSubjMesh, ClipInput subjInput,
	V3_F32 (* subjGetPos)(const void *, const void *, ClipInput, I32, I32),
	ClipFaceArr *pOut
) {
	//subject abbreviated to subj
	Result err = STUC_SUCCESS;
	ClipFuncs funcs = {.getClipPos = clipGetPos, .getSubjPos = subjGetPos};
	LinAlloc rootAlloc = {0};
	LinAlloc cornerAlloc = {0};
	I32 initSize = 0;
	stucClipInitMem(pAlloc, clipInput, subjInput, &rootAlloc, &cornerAlloc, &initSize);
	ClipFaceIntern clip = {0};
	ClipFaceIntern subj = {0};
	cornerListInit(
		&rootAlloc, &cornerAlloc,
		pUserData,
		pClipMesh, clipInput,
		callGetClipPos, &funcs,
		&clip,
		FACE_CLIP
	);
	cornerListInit(
		&rootAlloc, &cornerAlloc,
		pUserData,
		pSubjMesh, subjInput,
		callGetSubjPos, &funcs,
		&subj,
		FACE_SUBJECT
	);
	err = stucClipIntern(pAlloc, &cornerAlloc, initSize, &clip, &subj, pOut);
	STUC_THROW_IFNOT(err, "", 0);
	STUC_CATCH(0, err, ;);
	stucLinAllocDestroy(&rootAlloc);
	stucLinAllocDestroy(&cornerAlloc);
	return err;
}

void stucClipFaceArrDestroy(const StucAlloc *pAlloc, ClipFaceArr *pArr);