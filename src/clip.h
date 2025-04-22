#pragma once

#include "alloc.h"

#ifdef WIN32
#define PCUT_FORCE_INLINE __forceinline
#else
#define PCUT_FORCE_INLINE __attribute__((always_inline)) static inline
#endif

typedef PixtyV2_F32 PlycutV2_F32;
typedef PixtyV3_F32 PlycutV3_F32;
typedef PixalcFPtrs PlycutAlloc;
typedef PixErr PlycutErr;

typedef enum PlycutCornerType {
	PLYCUT_ORIGIN_CLIP,
	PLYCUT_ORIGIN_SUBJECT,
	PLYCUT_INTERSECT,
	PLYCUT_ON_CLIP_EDGE,
	PLYCUT_ON_SUBJECT_EDGE,
	PLYCUT_ON_VERT
} PlycutCornerType;

typedef struct PlycutCornerIdx {
	int32_t boundary;
	int32_t corner;
} PlycutCornerIdx;

typedef struct PlycutInfoOrigin {
	PlycutCornerIdx corner;
} PlycutInfoOrigin;

typedef struct PlycutInfoIntersect {
	PlycutCornerIdx clipCorner;
	PlycutCornerIdx subjCorner;
	float clipAlpha;
	float subjAlpha;
} PlycutInfoIntersect;

typedef struct PlycutInfoOnEdge {
	PlycutCornerIdx edgeCorner;
	PlycutCornerIdx vertCorner;
	float alpha;
} PlycutInfoOnEdge;

typedef struct PlycutInfoOnVert {
	PlycutCornerIdx clipCorner;
	PlycutCornerIdx subjCorner;
} PlycutInfoOnVert;

typedef union PlycutInfo {
	PlycutInfoOrigin origin;
	PlycutInfoIntersect intersect;
	PlycutInfoOnEdge onEdge;
	PlycutInfoOnVert onVert;
} PlycutInfo;

typedef struct PlycutCorner {
	struct PlycutCorner *pNext;
	struct PlycutCorner *pPrev;
	PlycutV3_F32 pos;
	PlycutCornerType type;
	PlycutInfo info;
} PlycutCorner;

typedef struct PlycutFaceRoot {
	PlycutCorner *pRoot;
	int32_t size;
} PlycutFaceRoot;

typedef struct PlycutFaceArr {
	PixalcLinAlloc cornerAlloc;
	PlycutFaceRoot *pArr;
	int32_t size;
	int32_t count;
} PlycutFaceArr;

typedef struct PlycutInput {
	int32_t *pSizes;
	int32_t boundaries;
	const void *pUserData;
} PlycutInput;

struct PlycutCornerIntern;

typedef struct PlycutFaceRootIntern {
	struct PlycutCornerIntern *pRoot;
	int32_t size;
	int32_t originSize;
	int32_t boundary;
} PlycutFaceRootIntern;

typedef struct PlycutFaceIntern {
	PlycutFaceRootIntern *pRoots;
	int32_t boundaries;
} PlycutFaceIntern;

typedef enum PlycutClipOrSubj {
	PCUT_FACE_CLIP,
	PCUT_FACE_SUBJECT
} PlycutClipOrSubj;

plycutClipIntern(
	const PlycutAlloc *pAlloc,
	PixalcLinAlloc *pCornerAlloc,
	int32_t initSize,
	PlycutFaceIntern *pClip, PlycutFaceIntern *pSubj,
	PlycutFaceArr *pOut
);

plycutClipInitMem(
	const PlycutAlloc *pAlloc,
	PlycutInput clipInput, PlycutInput subjInput,
	PixalcLinAlloc *pRootAlloc,
	PixalcLinAlloc *pCornerAlloc,
	int32_t *pInitSize
);

plycutClipInitCorner(
	PlycutFaceIntern *pFace,
	int32_t boundary,
	int32_t corner,
	PlycutClipOrSubj face,
	PlycutV3_F32 pos
);

typedef struct PlycutClipFuncs {
	PlycutV2_F32 (* getClipPos)(const void *, const void *, PlycutInput, int32_t, int32_t);
	PlycutV3_F32 (* getSubjPos)(const void *, const void *, PlycutInput, int32_t, int32_t);
} PlycutClipFuncs;

PCUT_FORCE_INLINE
PlycutV3_F32 plycutCallGetClipPos(
	const void *pUserData,
	const void *pMesh,
	const PlycutClipFuncs *pFuncs,
	PlycutInput inputFace,
	int32_t boundary,
	int32_t corner
) {
	PlycutV2_F32 pos = pFuncs->getClipPos(pUserData, pMesh, inputFace, boundary, corner);
	return (PlycutV3_F32) {.d = {pos.d[0], pos.d[1], .0f}};
}

PCUT_FORCE_INLINE
PlycutV3_F32 plycutCallGetSubjPos(
	const void *pUserData,
	const void *pMesh,
	const PlycutClipFuncs *pFuncs,
	PlycutInput inputFace,
	int32_t boundary,
	int32_t corner
) {
	return pFuncs->getSubjPos(pUserData, pMesh, inputFace, boundary, corner);
}

PCUT_FORCE_INLINE
void plycutCornerListInit(
	PixalcLinAlloc *pRootAlloc,
	PixalcLinAlloc *pCornerAlloc,
	const void *pUserData,
	const void *pMesh, PlycutInput inputFace,
	PlycutV3_F32 (* getPos)(const void *, const void *, const PlycutClipFuncs *, PlycutInput, int32_t, int32_t),
	const PlycutClipFuncs *pFuncs,
	PlycutFaceIntern *pFace,
	PlycutClipOrSubj face
) {
	pFace->boundaries = inputFace.boundaries;
	pixalcLinAlloc(pRootAlloc, &pFace->pRoots, pFace->boundaries);
	for (int32_t i = 0; i < inputFace.boundaries; ++i) {
		pFace->pRoots[i].size = inputFace.pSizes[i];
		pFace->pRoots[i].originSize = pFace->pRoots[i].size;
		pFace->pRoots[i].boundary = i;
		pixalcLinAlloc(pCornerAlloc, &pFace->pRoots[i].pRoot, inputFace.pSizes[i]);
		for (int32_t j = 0; j < inputFace.pSizes[i]; ++j) {
			PlycutV3_F32 pos = getPos(pUserData, pMesh, pFuncs, inputFace, i, j);
			plycutClipInitCorner(pFace, i, j, face, pos);
		}
	}
}

static inline
PlycutErr plycutClip(
	const PlycutAlloc *pAlloc,
	const void *pUserData,
	const void *pClipMesh, PlycutInput clipInput,
	PlycutV2_F32 (* clipGetPos)(const void *, const void *, PlycutInput, int32_t, int32_t),
	const void *pSubjMesh, PlycutInput subjInput,
	PlycutV3_F32 (* subjGetPos)(const void *, const void *, PlycutInput, int32_t, int32_t),
	PlycutFaceArr *pOut
) {
	//subject abbreviated to subj
	PlycutErr err = PIX_ERR_SUCCESS;
	PlycutClipFuncs funcs = {.getClipPos = clipGetPos, .getSubjPos = subjGetPos};
	PixalcLinAlloc rootAlloc = {0};
	PixalcLinAlloc cornerAlloc = {0};
	int32_t initSize = 0;
	plycutClipInitMem(pAlloc, clipInput, subjInput, &rootAlloc, &cornerAlloc, &initSize);
	PlycutFaceIntern clip = {0};
	PlycutFaceIntern subj = {0};
	plycutCornerListInit(
		&rootAlloc, &cornerAlloc,
		pUserData,
		pClipMesh, clipInput,
		plycutCallGetClipPos, &funcs,
		&clip,
		PCUT_FACE_CLIP
	);
	plycutCornerListInit(
		&rootAlloc, &cornerAlloc,
		pUserData,
		pSubjMesh, subjInput,
		plycutCallGetSubjPos, &funcs,
		&subj,
		PCUT_FACE_SUBJECT
	);
	err = plycutClipIntern(pAlloc, &cornerAlloc, initSize, &clip, &subj, pOut);
	PIX_ERR_THROW_IFNOT(err, "", 0);
	PIX_ERR_CATCH(0, err, ;);
	pixalcLinAllocDestroy(&rootAlloc);
	pixalcLinAllocDestroy(&cornerAlloc);
	return err;
}

void plycutFaceArrDestroy(const PlycutAlloc *pAlloc, PlycutFaceArr *pArr);