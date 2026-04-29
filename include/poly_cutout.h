/* 
SPDX-FileCopyrightText: 2025 Caleb Dawson
SPDX-License-Identifier: Apache-2.0
*/

/*
implements Foster-Hormann-Popa clipping:
Clipping simple polygons with degenerate intersections, Foster et al, 2019
https://www.inf.usi.ch/hormann/papers/Foster.2019.CSP.pdf

Foster-Hormann-Popa modifies Greiner-Hormann clipping to properly handle degen cases.
original Greiner-Hormann paper:
Efficient clipping of arbitrary polygons, Greiner et al, 1998
https://www.inf.usi.ch/hormann/papers/Greiner.1998.ECO.pdf
*/

#pragma once

#include <float.h>

#include <pixenals_alloc_utils.h>

#ifdef NDEBUG
#ifdef WIN32
#define PLYCUT_FORCE_INLINE __forceinline
#else
#define PLYCUT_FORCE_INLINE __attribute__((always_inline)) static inline
#endif
#else
#define PLYCUT_FORCE_INLINE static inline
#endif

#ifndef PLYCUT_SNAP_THRESHOLD
	#define PLYCUT_SNAP_THRESHOLD .0f
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

typedef struct PlycutCornerUserData {
	uint32_t clip;
	uint32_t subj;
} PlycutCornerUserData;

typedef struct PlycutCorner {
	struct PlycutCorner *pNext;
	struct PlycutCorner *pPrev;
	PlycutV3_F32 pos;
	PlycutCornerType type;
	PlycutInfo info;
	PlycutCornerUserData userData;
	bool cross;
} PlycutCorner;

typedef struct PlycutFaceRoot {
	PlycutCorner *pRoot;
	int32_t size;
	bool isHole;
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
	int32_t crossCount;
	int32_t boundary;
	bool commonEdges;
	bool skip;
	bool in;
	bool modified;
} PlycutFaceRootIntern;

typedef struct PlycutBb {
	PixtyV2_F32 min;
	PixtyV2_F32 max;
} PlycutBb;

typedef struct PlycutFaceIntern {
	PlycutFaceRootIntern *pRoots;
	int32_t boundaries;
	PlycutBb bb;
} PlycutFaceIntern;

typedef struct PlycutAlloc {
	PixalcLinAlloc root;
	PixalcLinAlloc corner;
} PlycutMem;

typedef enum PlycutClipOrSubj {
	PLYCUT_FACE_CLIP,
	PLYCUT_FACE_SUBJECT
} PlycutClipOrSubj;

PlycutErr plycutClipIntern(
	const PlycutAlloc *pAlloc,
	PixalcLinAlloc *pCornerAlloc,
	int32_t initSize,
	PlycutFaceIntern *pClip, PlycutFaceIntern *pSubj,
	PlycutFaceArr *pOut,
	bool *pOverlap
);

void plycutClipInitMem(
	const PlycutAlloc *pAlloc,
	PlycutInput clipInput, PlycutInput subjInput,
	PixalcLinAlloc *pRootAlloc,
	PixalcLinAlloc *pCornerAlloc,
	int32_t *pInitSize
);

void plycutClipInitCorner(
	PlycutFaceIntern *pFace,
	int32_t boundary,
	int32_t corner,
	PlycutClipOrSubj face,
	PlycutV3_F32 pos,
	bool cantIntersect,
	uint32_t userData
);

typedef struct PlycutClipFuncs {
	PlycutV2_F32 (* getClipPos)(
		const void *, void *, PlycutInput, int32_t, int32_t, bool *, uint32_t *
	);
	PlycutV3_F32 (* getSubjPos)(
		const void *, void *, PlycutInput, int32_t, int32_t, bool *, uint32_t *
	);
} PlycutClipFuncs;

PLYCUT_FORCE_INLINE
PlycutV3_F32 plycutCallGetClipPos(
	const void *pUserData,
	void *pMesh,
	const PlycutClipFuncs *pFuncs,
	PlycutInput inputFace,
	int32_t boundary,
	int32_t corner,
	bool *pCantIntersect,
	uint32_t *pCornerUserData
) {
	PlycutV2_F32 pos = pFuncs->getClipPos(
		pUserData,
		pMesh,
		inputFace,
		boundary,
		corner,
		pCantIntersect,
		pCornerUserData
	);
	return (PlycutV3_F32) {.d = {pos.d[0], pos.d[1], .0f}};
}

PLYCUT_FORCE_INLINE
PlycutV3_F32 plycutCallGetSubjPos(
	const void *pUserData,
	void *pMesh,
	const PlycutClipFuncs *pFuncs,
	PlycutInput inputFace,
	int32_t boundary,
	int32_t corner,
	bool *pCantIntersect,
	uint32_t *pCornerUserData
) {
	return pFuncs->getSubjPos(
		pUserData,
		pMesh,
		inputFace,
		boundary,
		corner,
		pCantIntersect,
		pCornerUserData
	);
}

PLYCUT_FORCE_INLINE
void plycutCornerListInit(
	PixalcLinAlloc *pRootAlloc,
	PixalcLinAlloc *pCornerAlloc,
	const void *pUserData,
	void *pMesh, PlycutInput inputFace,
	PlycutV3_F32 (* getPos)(
		const void *,
		void *,
		const PlycutClipFuncs *,
		PlycutInput,
		int32_t,
		int32_t,
		bool *,
		uint32_t *
	),
	const PlycutClipFuncs *pFuncs,
	PlycutFaceIntern *pFace,
	PlycutClipOrSubj face
) {
	*pFace = (PlycutFaceIntern){
		.boundaries = inputFace.boundaries,
		.bb = {
			.min = (PixtyV2_F32){.d = {FLT_MAX, FLT_MAX}},
			.max = (PixtyV2_F32){.d = {-FLT_MAX, -FLT_MAX}}
		}
	};
	pixalcLinAlloc(pRootAlloc, (void **)&pFace->pRoots, pFace->boundaries);
	for (int32_t i = 0; i < inputFace.boundaries; ++i) {
		pFace->pRoots[i].size = inputFace.pSizes[i];
		pFace->pRoots[i].originSize = pFace->pRoots[i].size;
		pFace->pRoots[i].boundary = i;
		pixalcLinAlloc(
			pCornerAlloc,
			(void **)&pFace->pRoots[i].pRoot,
			inputFace.pSizes[i]
		);
		for (int32_t j = 0; j < inputFace.pSizes[i]; ++j) {
			bool cantIntersect = false;
			uint32_t userData = 0u;
			PlycutV3_F32 pos = getPos(
				pUserData,
				pMesh,
				pFuncs,
				inputFace,
				i, j,
				&cantIntersect,
				&userData
			);
			plycutClipInitCorner(pFace, i, j, face, pos, cantIntersect, userData);
		}
	}
}

static inline
bool plycutDoBbOverlap(PlycutBb a, PlycutBb b) {
	return
		a.min.d[0] <= b.max.d[0] && a.max.d[0] >= b.min.d[0] &&
		a.min.d[1] <= b.max.d[1] && a.max.d[1] >= b.min.d[1];
}

static inline
PlycutErr plycutClip(
	const PlycutAlloc *pAlloc,
	const void *pUserData,
	void *pClipMesh, PlycutInput clipInput,
	PlycutV2_F32 (* clipGetPos)(
		const void *,
		void *,
		PlycutInput,
		int32_t,
		int32_t,
		bool *,
		uint32_t *
	),
	void *pSubjMesh, PlycutInput subjInput,
	PlycutV3_F32 (* subjGetPos)(
		const void *,
		void *,
		PlycutInput,
		int32_t,
		int32_t,
		bool *,
		uint32_t *
	),
	PlycutFaceArr *pOut,
	bool *pOverlap,
	PlycutMem *pLinAlc //will reuse existing mem if this isn't null
) {
	PIX_ERR_ASSERT(
		"one alloc handle has been used, but not the other",
		!(pLinAlc->root.valid ^ pLinAlc->corner.valid)
	);
	bool reuseMem = pLinAlc->root.valid;
	//subject abbreviated to subj
	PlycutErr err = PIX_ERR_SUCCESS;
	PIX_ERR_RETURN_IFNOT_COND(err, !pOut ^ !pOverlap, "Either pOut or pOverlap must be non-null");
	PlycutClipFuncs funcs = {.getClipPos = clipGetPos, .getSubjPos = subjGetPos};
	PixalcLinAlloc rootAlloc = {0};
	PixalcLinAlloc cornerAlloc = {0};
	PixalcLinAlloc *pRootAlc = pLinAlc ? &pLinAlc->root : &rootAlloc;
	PixalcLinAlloc *pCornerAlc = pLinAlc ? &pLinAlc->corner : &cornerAlloc;
	int32_t initSize = 0;
	plycutClipInitMem(
		pAlloc,
		clipInput,
		subjInput,
		reuseMem ? NULL : pRootAlc,
		reuseMem ? NULL : pCornerAlc,
		&initSize
	);
	PlycutFaceIntern clip = {0};
	PlycutFaceIntern subj = {0};
	plycutCornerListInit(
		pRootAlc, pCornerAlc,
		pUserData,
		pClipMesh, clipInput,
		plycutCallGetClipPos, &funcs,
		&clip,
		PLYCUT_FACE_CLIP
	);
	plycutCornerListInit(
		pRootAlc, pCornerAlc,
		pUserData,
		pSubjMesh, subjInput,
		plycutCallGetSubjPos, &funcs,
		&subj,
		PLYCUT_FACE_SUBJECT
	);
	if (!plycutDoBbOverlap(clip.bb, subj.bb)) {
		//TODO does this make bounds checking in uv-stucco redundant?
		if (pOverlap) {
			*pOverlap = false;
		}
		return err;
	}
	err = plycutClipIntern(pAlloc, pCornerAlc, initSize, &clip, &subj, pOut, pOverlap);
	PIX_ERR_THROW_IFNOT(err, "", 0);
	PIX_ERR_CATCH(0, err, ;);
	if (pLinAlc) {
		pixalcLinAllocClear(pRootAlc);
		pixalcLinAllocClear(pCornerAlc);
	}
	else {
		pixalcLinAllocDestroy(pRootAlc);
		pixalcLinAllocDestroy(pCornerAlc);
	}
	return err;
}

static inline
void plycutMemDestroy(PlycutMem *pMem) {
	PIX_ERR_ASSERT("", pMem);
	if (pMem->root.valid) {
		pixalcLinAllocDestroy(&pMem->root);
	}
	if (pMem->corner.valid) {
		pixalcLinAllocDestroy(&pMem->corner);
	}
}

void plycutFaceArrDestroy(const PlycutAlloc *pAlloc, PlycutFaceArr *pArr);
