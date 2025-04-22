#include <float.h>
#include <math.h>

#include <clip.h>
#include <math_utils.h>

typedef int8_t I8;
typedef int16_t I16;
typedef int32_t I32;
typedef int64_t I64;

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;

typedef float F32;
typedef double F64;

typedef uint8_t UBitField8;
typedef uint16_t UBitField16;
typedef uint32_t UBitField32;
typedef uint64_t UBitField64;

typedef PixtyV2_F32 V2_F32;
typedef PixtyV3_F32 V3_F32;

typedef PixtyRange Range;

typedef PixtyI32Arr I32Arr;
typedef PixtyI8Arr I8Arr;

typedef PlycutFaceRootIntern FaceRootIntern;
typedef PlycutFaceIntern FaceIntern;


#define SNAP_THRESHOLD .0001f

typedef enum Label {
	LABEL_NONE,
	LABEL_CROSS,
	LABEL_CROSS_DELAYED,
	LABEL_CROSS_CANDIDATE,
	LABEL_BOUNCE,
	LABEL_BOUNCE_DELAYED,
	LABEL_LEFT_ON,
	LABEL_RIGHT_ON,
	LABEL_ON_ON,
	LABEL_ON_LEFT,
	LABEL_ON_RIGHT
} Label;

typedef enum CrossDir {
	CROSS_NONE,
	CROSS_ENTRY,
	CROSS_EXIT
} CrossDir;

typedef struct PlycutCornerIntern {
	struct PlycutCornerIntern *pNext;
	struct PlycutCornerIntern *pPrev;
	struct PlycutCornerIntern *pNextOrigin;
	struct PlycutCornerIntern *pPrevOrigin;
	struct PlycutCornerIntern *pLink;//using 'link' to refer to an intersect or overlap point
	V3_F32 pos;
	I32 originCorner;
	I32 boundary;
	F32 alpha;
	Label label;
	CrossDir travel;
	PlycutClipOrSubj face;
	bool checked;
	bool original;
	bool cross;
	bool dontAdd;
} PlycutCornerIntern;

typedef PlycutCornerIntern Corner;

typedef struct FaceIter {
	void *pUserData;
	FaceIntern *pFace;
	Corner *pCorner;
	Corner *pStart;
	PixErr (* fpStartPredicate)(void *, const Corner *, bool *);
	PixErr (* fpHandleNoStart)(void *, FaceRootIntern *, Corner **);
	I32 boundary;
	I32 corner;
	I32 count;
	PixErr err;
	bool originOnly;
	bool skipBoundary;
} FaceIter;

static
void faceIterInit(
	FaceIntern *pFace,
	void *pUserData,
	PixErr (* fpStartPredicate)(void *, const Corner *, bool *),
	PixErr (* fpHandleNoStart)(void *, FaceRootIntern *, Corner **),
	bool originOnly,
	FaceIter *pIter
	) {
	*pIter = (FaceIter) {
		.pUserData = pUserData,
		.pFace = pFace,
		.err = PIX_ERR_SUCCESS,
		.fpStartPredicate = fpStartPredicate,
		.fpHandleNoStart = fpHandleNoStart,
		.originOnly = originOnly
	};
}

static
void faceIterInternIncCorner(FaceIter *pIter) {
	pIter->pCorner = pIter->originOnly ?
		pIter->pCorner->pNextOrigin : pIter->pCorner->pNext;
}

static
I32 faceIterInternGetSize(const FaceIter *pIter) {
	return pIter->originOnly ?
		pIter->pFace->pRoots[pIter->boundary].originSize :
		pIter->pFace->pRoots[pIter->boundary].size;
}

static
PixErr faceIterInternFindValidStart(FaceIter *pIter) {
	PixErr err = PIX_ERR_SUCCESS;
	I32 i = 0;
	bool valid = false;
	I32 size = faceIterInternGetSize(pIter);
	while (
		err = pIter->fpStartPredicate(pIter->pUserData, pIter->pCorner, &valid),
		!valid
	) {
		if (err != PIX_ERR_SUCCESS) {
			break;
		}
		if (i >= size) {
			PIX_ERR_ASSERT("", i == size);
			PIX_ERR_RETURN_IFNOT_COND(
				err,
				pIter->fpHandleNoStart,
				"no valid start corner found"
			);
			FaceRootIntern *pRoot = pIter->pFace->pRoots + pIter->boundary;
			err = pIter->fpHandleNoStart(pIter->pUserData, pRoot, &pIter->pCorner);
			PIX_ERR_RETURN_IFNOT(err, "");
			if (pIter->pCorner) {
				err = pIter->fpStartPredicate(pIter->pUserData, pIter->pCorner, &valid);
				PIX_ERR_RETURN_IFNOT(err, "");
				PIX_ERR_ASSERT(
					"'no start' handler gave start corner that doesn't pass predicate",
					valid
				);
			}
			if (!valid) {
			}
			return err;
		}
		faceIterInternIncCorner(pIter);
		++i;
	}
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
bool faceIterAtEnd(const FaceIter *pIter) {
	return
		pIter->err != PIX_ERR_SUCCESS ||
		pIter->boundary >= pIter->pFace->boundaries ||
		pIter->boundary == pIter->pFace->boundaries - 1 &&
		pIter->corner >= faceIterInternGetSize(pIter);
}

static
bool faceIterSetCorner(FaceIter *pIter) {
	if (faceIterAtEnd(pIter)) {
		return true;
	}
	if (pIter->corner) {
		faceIterInternIncCorner(pIter);
	}
	else {
		FaceRootIntern *pRoot = pIter->pFace->pRoots + pIter->boundary;
		pIter->pCorner = pRoot->pRoot;
		if (pIter->fpStartPredicate) {
			pIter->err = faceIterInternFindValidStart(pIter);
			PIX_ERR_RETURN_IFNOT(pIter->err, "");
		}
		pIter->pStart = pIter->pCorner;
	}
	return false;
	PIX_ERR_CATCH(0, pIter->err, ;);
	return true;
}

static
void faceIterInc(FaceIter *pIter) {
	PIX_ERR_ASSERT("", pIter->boundary < pIter->pFace->boundaries);
	++pIter->corner;
	if (!pIter->pStart || pIter->corner >= faceIterInternGetSize(pIter)) {
		if (!pIter->pStart) {
			PIX_ERR_ASSERT("start is null, but corner isn't?", !pIter->pCorner);
		}
		else {
			Corner *pCornerNext = pIter->originOnly ?
				pIter->pCorner->pNextOrigin : pIter->pCorner->pNext;
			PIX_ERR_THROW_IFNOT_COND(
				pIter->err,
				pCornerNext == pIter->pStart,
				"infinite or astray loop",
				0
			);
		}
		++pIter->boundary;
		pIter->corner = 0;
	}
	++pIter->count;
	PIX_ERR_CATCH(0, pIter->err, ;);
	return;
}

static
void faceIterIncBoundary(FaceIter *pIter) {
	pIter->pStart = pIter->pCorner = NULL;
	faceIterInc(pIter);
}

static
FaceRootIntern *faceIterGetRoot(FaceIter *pIter) {
	PIX_ERR_ASSERT("", pIter->boundary < pIter->pFace->boundaries);
	return pIter->pFace->pRoots + pIter->boundary;
}

static
PixErr faceIterGetErr(const FaceIter *pIter) {
	return pIter->err;
}

static
F32 getSignedArea(V2_F32 a, V2_F32 b, V2_F32 c) {
	return _(_(b V2SUB a) V2CROSS _(c V2SUB a));
}

//returns 1 if edges are parallel, 2 if colinear
static
I32 getIntersectAlpha(V3_F32 a, V3_F32 b, V3_F32 c, V3_F32 d, F32 *pAlpha) {
	F32 acd = getSignedArea(*(V2_F32 *)&a, *(V2_F32 *)&c, *(V2_F32 *)&d);
	F32 bcd = getSignedArea(*(V2_F32 *)&b, *(V2_F32 *)&c, *(V2_F32 *)&d);
	F32 cdLen = pixmV2F32Len(_(*(V2_F32 *)&d V2SUB *(V2_F32 *)&c));
	F32 hAcd = acd / cdLen;
	F32 hBcd = bcd / cdLen;
	F32 diff = fabsf(hAcd - hBcd);
	bool aIsOnCd = _(fabsf(hAcd) F32_LESS SNAP_THRESHOLD);
	if (_(diff F32_LESS SNAP_THRESHOLD)) {
		return aIsOnCd ? 2 : 1;
	}
	if (aIsOnCd) {
		*pAlpha = .0f;
	}
	else if (_(fabsf(hBcd) F32_LESS SNAP_THRESHOLD)) {
		*pAlpha = 1.0f;
	}
	else {
		F32 divisor = acd - bcd;
		PIX_ERR_ASSERT("", _(divisor F32_NOTEQL .0f));
		*pAlpha = acd / divisor;
	}
	return 0;
}

typedef enum Intersect {
	INTERSECT_NONE,
	INTERSECT_CROSS,
	INTERSECT_T,
	INTERSECT_V
} Intersect;

static
PixErr insertCorner(FaceRootIntern *pRoot, Corner *pCorner, Corner *pNew, bool makeOriginal) {
	PixErr err = PIX_ERR_SUCCESS;
	if (!makeOriginal) {
		while (
			!pCorner->pNext->original &&
			_(pNew->alpha F32_GREAT pCorner->pNext->alpha)
		) {
			pCorner = pCorner->pNext;
			PIX_ERR_RETURN_IFNOT_COND(
				err,
				_(pNew->alpha F32_NOTEQL pCorner->alpha),
				"degen verts"
			);
		}
	}
	pNew->pNext = pCorner->pNext;
	pCorner->pNext = pNew;
	pNew->pNext->pPrev = pNew;
	pNew->pPrev = pCorner;

	pNew->face = pCorner->face;
	pNew->original = makeOriginal;
	pNew->boundary = pRoot->boundary;
	pNew->originCorner = pCorner->originCorner;
	++pRoot->size;

	pNew->pNextOrigin = pCorner->pNextOrigin;
	pNew->pPrevOrigin = pCorner->original ? pCorner : pCorner->pPrevOrigin;
	if (makeOriginal) {
		pCorner->pNextOrigin = pNew;
		pNew->pNextOrigin->pPrevOrigin = pNew;
		++pRoot->originSize;
	}
	return err;
}

static
void linkCorners(Corner *pA, Corner *pB) {
	PIX_ERR_ASSERT("trying to link corners(s) with existing link", !pA->pLink && !pB->pLink);
	pA->pLink = pB;
	pB->pLink = pA;
}

static
PixErr insertT(PixalcLinAlloc *pAlloc, FaceRootIntern *pRoot, Corner *pEdge, F32 aEdge, Corner *pPoint) {
	PixErr err = PIX_ERR_SUCCESS;
	Corner *pCopy = NULL;
	pixalcLinAlloc(pAlloc, &pCopy, 1);
	*pCopy = *pPoint;
	pCopy->alpha = aEdge;
	linkCorners(pCopy, pPoint);
	err = insertCorner(pRoot, pEdge, pCopy, false);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
PixErr handleXIntersect(
	PixalcLinAlloc *pAlloc,
	FaceRootIntern *pClipRoot, FaceRootIntern *pSubjRoot,
	Corner *pClip, Corner *pSubj,
	F32 aClipEdge, F32 aSubjEdge
) {
	PixErr err = PIX_ERR_SUCCESS;
	Corner *pIntersect = NULL;
	pixalcLinAlloc(pAlloc, &pIntersect, 2);//2 copies for each list
	//subject face is 3D (clip face is 2D), so we use it to calc interesction
	pIntersect[0].pos = _(pSubj->pos V3ADD _(
		_(pSubj->pNextOrigin->pos V3SUB pSubj->pos) V3MULS aSubjEdge)
	);
	pIntersect[1] = *pIntersect;
	pIntersect[0].alpha = aClipEdge;
	pIntersect[1].alpha = aSubjEdge;
	linkCorners(pIntersect, pIntersect + 1);
	err = insertCorner(pClipRoot, pClip, pIntersect, false);
	PIX_ERR_RETURN_IFNOT(err, "");
	err = insertCorner(pSubjRoot, pSubj, pIntersect + 1, false);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
F32 getColinearAlpha(V2_F32 a, V2_F32 b, V2_F32 c) {
	PIX_ERR_ASSERT("", _(a V2NOTEQL b));
	V2_F32 ab = _(b V2SUB a);
	V2_F32 ac = _(c V2SUB a);
	//assuming here that a != b, degen edge check should have caught that prior
	F32 alpha =  _(ac V2DOT ab) / _(ab V2DOT ab);
	F32 abLen = pixmV2F32Len(ab);
	if (_(fabsf(alpha * abLen) F32_LESS SNAP_THRESHOLD)) {
		alpha = .0f;
	}
	else if (_(fabsf((1.0f - alpha) * abLen) F32_LESS SNAP_THRESHOLD)) {
		alpha = 1.0f;
	}
	return alpha;
}

static
PixErr insertIntersect(
	PixalcLinAlloc *pAlloc,
	FaceRootIntern *pClipRoot, FaceRootIntern *pSubjRoot,
	Corner *pClip, Corner *pSubj,
	bool *pColinear
) {
	PixErr err = PIX_ERR_SUCCESS;
	F32 aClipEdge = .0f;
	F32 aSubjEdge = .0f;
	V3_F32 clipPosNext = pClip->pNextOrigin->pos;
	V3_F32 subjPosNext = pSubj->pNextOrigin->pos;
	I32 resultA =
		getIntersectAlpha(pSubj->pos, subjPosNext, pClip->pos, clipPosNext, &aSubjEdge);
	I32 resultB = 
		getIntersectAlpha(pClip->pos, clipPosNext, pSubj->pos, subjPosNext, &aClipEdge);
	if (resultA || resultB) {
		if (resultA == 2 && resultB == 2) {
			*pColinear = true;
		}
		return err;
	}
	//alphas are set to 0 or 1 in getIntersectAlpha, if within snap threshold.
	// so not using epsilon here
	if (aClipEdge > .0f && aClipEdge < 1.0f &&
		aSubjEdge > .0f && aSubjEdge < 1.0f
	) {
		//X intersection
		err = handleXIntersect(
			pAlloc,
			pClipRoot, pSubjRoot,
			pClip, pSubj,
			aClipEdge, aSubjEdge
		);
		PIX_ERR_RETURN_IFNOT(err, "");
	}
	else if (!aClipEdge && !aSubjEdge) {
		//V intersection
		PIX_ERR_RETURN_IFNOT_COND(err, !pClip->pLink && !pSubj->pLink, "degen verts");
		linkCorners(pClip, pSubj);
	}
	else if (!aSubjEdge) {
		aClipEdge = getColinearAlpha(
			*(V2_F32 *)&pClip->pos, *(V2_F32 *)&pClip->pNextOrigin->pos,
			*(V2_F32 *)&pSubj->pos
		);
		if (!aClipEdge) {
			//V intersection
			PIX_ERR_RETURN_IFNOT_COND(err, !pClip->pLink && !pSubj->pLink, "degen verts");
			linkCorners(pClip, pSubj);
		}
		else if (aClipEdge > .0f && aClipEdge < 1.0f) {
			//T intersection on clip edge
			err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
			PIX_ERR_RETURN_IFNOT(err, "");
		}
	}
	else if (!aClipEdge) {
		aSubjEdge = getColinearAlpha(
			*(V2_F32 *)&pSubj->pos, *(V2_F32 *)&pSubj->pNextOrigin->pos,
			*(V2_F32 *)&pClip->pos
		);
		if (!aSubjEdge) {
			//V intersection
			PIX_ERR_RETURN_IFNOT_COND(err, !pClip->pLink && !pSubj->pLink, "degen verts");
			linkCorners(pClip, pSubj);
		}
		else if (aSubjEdge > .0f && aSubjEdge < 1.0f) {
			//T intersection on subject edge
			err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
			PIX_ERR_RETURN_IFNOT(err, "");
		}
	}
	return err;
}

static
PixErr insertOverlap(
	PixalcLinAlloc *pAlloc,
	FaceRootIntern *pClipRoot, FaceRootIntern *pSubjRoot,
	Corner *pClip, Corner *pSubj
) {
	PixErr err = PIX_ERR_SUCCESS;
	F32 aClipEdge = getColinearAlpha(
		*(V2_F32 *)&pClip->pos, *(V2_F32 *)&pClip->pNextOrigin->pos,
		*(V2_F32 *)&pSubj->pos
	);
	F32 aSubjEdge = getColinearAlpha(
		*(V2_F32 *)&pSubj->pos, *(V2_F32 *)&pSubj->pNextOrigin->pos,
		*(V2_F32 *)&pClip->pos
	);
	//getColinearAlpha sets alpha to 0 or 1, if within snap threshold.
	// so not using epsilon here
	bool aClipIsIn01 = aClipEdge > .0f && aClipEdge < 1.0f;
	bool aSubjIsIn01 = aSubjEdge > .0f && aSubjEdge < 1.0f;
	if (aClipIsIn01 && aSubjIsIn01) {
		//X overlap
		err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
		PIX_ERR_RETURN_IFNOT(err, "");
		err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
		PIX_ERR_RETURN_IFNOT(err, "");
	}
	else if (aClipIsIn01 && (!aSubjIsIn01 || aSubjEdge == 1.0f)) {
		//T overlap on clip edge
		err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
		PIX_ERR_RETURN_IFNOT(err, "");
	}
	else if (aSubjIsIn01 && (!aClipIsIn01 || aClipEdge == 1.0f)) {
		//T overlap on subj edge
		err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
		PIX_ERR_RETURN_IFNOT(err, "");
	}
	else if (!aClipEdge && !aSubjEdge) {
		//V overlap
		linkCorners(pClip, pSubj);
	}
	return err;
}

static
PixErr intersectHalfEdges(
	PixalcLinAlloc *pAlloc, 
	FaceRootIntern *pClipRoot, FaceRootIntern *pSubjRoot,
	Corner *pClip, Corner *pSubj
) {
	PixErr err = PIX_ERR_SUCCESS;
	PIX_ERR_RETURN_IFNOT_COND(
		err,
		_(*(V2_F32 *)&pClip->pos V2NOTEQL * (V2_F32 *)&pClip->pNextOrigin->pos) &&
		_(*(V2_F32 *)&pSubj->pos V2NOTEQL * (V2_F32 *)&pSubj->pNextOrigin->pos),
		"degen edge(s)"
	);
	bool colinear = false;
	err = insertIntersect(pAlloc, pClipRoot, pSubjRoot, pClip, pSubj, &colinear);
	PIX_ERR_RETURN_IFNOT(err, "");
	if (colinear) {
		//colinear
		insertOverlap(pAlloc, pClipRoot, pSubjRoot, pClip, pSubj);
	}
	return err;
}

typedef enum Hand {
	HAND_STRAIGHT,
	HAND_LEFT,
	HAND_RIGHT,
	HAND_ON
} Hand;

typedef enum Neighbour {
	NEIGHBOUR_PREV,
	NEIGHBOUR_NEXT
} Neighbour;

typedef struct LocalInfo {
	Hand turnSNext;
	Hand turnCPrev;
	Hand turnCNext;
	Neighbour sPrevLink;
	Neighbour sNextLink;
	bool sPrevOnC;
	bool sNextOnC;
} LocalInfo;

static
bool isLinkWithPrevOrNext(
	const Corner *pA,
	const Corner *pBPrev, const Corner *pBNext,
	Neighbour *pWhich
) {
	if (pA->pLink) {
		if (pA->pLink == pBPrev) {
			*pWhich = NEIGHBOUR_PREV;
			return true;
		}
		else if (pA->pLink == pBNext) {
			*pWhich = NEIGHBOUR_NEXT;
			return true;
		}
	}
	return false;
}

static
LocalInfo getLocalInfoForIntersect(const Corner *pClip, const Corner *pSubj) {
	V2_F32 sPrev = *(V2_F32 *)&pSubj->pPrev->pos;
	V2_F32 point = *(V2_F32 *)&pSubj->pos;
	V2_F32 sNext = *(V2_F32 *)&pSubj->pNext->pos;
	F32 signSNext = getSignedArea(sPrev, point, sNext);
	LocalInfo info = {
		.turnSNext = _(signSNext F32_EQL .0f) ? HAND_STRAIGHT :
			_(signSNext F32_GREAT .0f) ? HAND_LEFT : HAND_RIGHT,
	};
	info.sPrevOnC =
		isLinkWithPrevOrNext(pSubj->pPrev, pClip->pPrev, pClip->pNext, &info.sPrevLink);
	info.sNextOnC =
		isLinkWithPrevOrNext(pSubj->pNext, pClip->pPrev, pClip->pNext, &info.sNextLink);
	V2_F32 cPrev = *(V2_F32 *)&pClip->pPrev->pos;
	V2_F32 cNext = *(V2_F32 *)&pClip->pNext->pos;
	F32 signCPrev_0 = getSignedArea(cPrev, sPrev, point);
	F32 signCPrev_1 = getSignedArea(cPrev, point, sNext);
	F32 signCNext_0 = getSignedArea(cNext, sPrev, point);
	F32 signCNext_1 = getSignedArea(cNext, point, sNext);
	switch (info.turnSNext) {
		case HAND_STRAIGHT:
		case HAND_LEFT:
			info.turnCPrev = _(signCPrev_0 F32_LESS .0f) || _(signCPrev_1 F32_LESS .0f) ? 
				HAND_RIGHT : HAND_LEFT;
			info.turnCNext = _(signCNext_0 F32_LESS .0f) || _(signCNext_1 F32_LESS .0f) ?
				HAND_RIGHT : HAND_LEFT;
			break;
		case HAND_RIGHT:
			info.turnCPrev = _(signCPrev_0 F32_GREAT .0f) || _(signCPrev_1 F32_GREAT .0f) ? 
				HAND_LEFT : HAND_RIGHT;
			info.turnCNext = _(signCNext_0 F32_GREAT .0f) || _(signCNext_1 F32_GREAT .0f) ?
				HAND_LEFT : HAND_RIGHT;
	}
	return info;
}

static
void setLinkLabel(Corner *pCorner, Label label) {
	PIX_ERR_ASSERT("", pCorner->pLink);
	pCorner->pLink->label = pCorner->label = label;
}

static
void setLinkCross(Corner *pCorner) {
	PIX_ERR_ASSERT("", pCorner->pLink);
	pCorner->pLink->cross = pCorner->cross = true;
}

static
PixErr noOnLeftRightPredicate(void *pUserData, const Corner *pCorner, bool *pValid) {
	*pValid =
		pCorner->label != LABEL_ON_ON &&
		pCorner->label != LABEL_ON_LEFT &&
		pCorner->label != LABEL_ON_RIGHT;
	return PIX_ERR_SUCCESS;
}

static
PixErr labelCrossOrBounce(FaceIntern *pSubjFace) {
	PixErr err = PIX_ERR_SUCCESS;
	FaceIter subjIter = {0};
	faceIterInit(pSubjFace, NULL, NULL, NULL, false, &subjIter);
	for (; !faceIterSetCorner(&subjIter); faceIterInc(&subjIter)) {
		if (!subjIter.pCorner->pLink) {
			continue;
		}
		Corner *pClip = subjIter.pCorner->pLink;
		LocalInfo info = getLocalInfoForIntersect(pClip, subjIter.pCorner);
		Label label = LABEL_NONE;
		if (info.sPrevOnC && info.sNextOnC) {
			//both edges overlap
			label = LABEL_ON_ON;
		}
		else if (info.sPrevOnC) {
			//subject-,subject overlaps with clip,clip+ or clip-,clip
			Hand cTurn = info.sPrevLink == NEIGHBOUR_PREV ?
				info.turnCNext : info.turnCPrev;
			label = cTurn == HAND_RIGHT ? LABEL_ON_LEFT : LABEL_ON_RIGHT;
		}
		else if (info.sNextOnC) {
			//subject,subject+ overlaps with clip,clip+ or clip-,clip
			Hand cTurn = info.sNextLink == NEIGHBOUR_PREV ?
				info.turnCNext : info.turnCPrev;
			label = cTurn == HAND_RIGHT ? LABEL_LEFT_ON : LABEL_RIGHT_ON;
		}
		else if (info.turnCPrev == info.turnCNext) {
			label = LABEL_BOUNCE;
		}
		else {
			label = LABEL_CROSS;
			setLinkCross(subjIter.pCorner);
		}
		setLinkLabel(subjIter.pCorner, label);
	}
	err = faceIterGetErr(&subjIter);
	PIX_ERR_RETURN_IFNOT(err, "");

	typedef struct OverlapChain {
		Corner *pStart;
	} OverlapChain;
	OverlapChain chain = {0};
	faceIterInit(pSubjFace, NULL, noOnLeftRightPredicate, NULL, false, &subjIter);
	for (; !faceIterSetCorner(&subjIter); faceIterInc(&subjIter)) {
		Label label = subjIter.pCorner->label;
		if (!subjIter.pCorner->pLink || label == LABEL_CROSS || label == LABEL_BOUNCE) {
			PIX_ERR_ASSERT("", !chain.pStart);
			continue;
		}
		if (!chain.pStart) {
			PIX_ERR_ASSERT("", label == LABEL_LEFT_ON || label == LABEL_RIGHT_ON);
			chain.pStart = subjIter.pCorner;
		}
		else if (label != LABEL_ON_ON){
			PIX_ERR_ASSERT("", label == LABEL_ON_LEFT || label == LABEL_ON_RIGHT);
			label = (chain.pStart->label == LABEL_LEFT_ON) == (label == LABEL_ON_LEFT) ?
				LABEL_BOUNCE_DELAYED : LABEL_CROSS_DELAYED;
			setLinkLabel(subjIter.pCorner, label);
			setLinkLabel(chain.pStart, label);
			chain.pStart = NULL;
		}
	}
	err = faceIterGetErr(&subjIter);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
PixErr inTestStartPredicate(void *pUserData, const Corner *pCorner, bool *pValid) {
	F32 diff = pCorner->pos.d[0] - ((V2_F32 *)pUserData)->d[0];
	*pValid = _(fabsf(diff) F32_GREAT SNAP_THRESHOLD * 4.0f);
	return PIX_ERR_SUCCESS;
}

static
PixErr inTestNoStartHandler(void *pUserData, FaceRootIntern *pRoot, Corner **ppStart) {
	*ppStart = NULL;
	return PIX_ERR_SUCCESS;
}

static
bool doesIntersectRay(V2_F32 rayA, V2_F32 rayB, V2_F32 c, V2_F32 d) {
	F32 acd = getSignedArea(rayA, c, d);
	F32 bcd = getSignedArea(rayB, c, d);
	F32 divisor = acd - bcd;
	if (_(divisor F32_EQL .0f)) {
		return false;
	}
	return _(acd / divisor F32_GREATEQL .0f);
	//F32 abAlpha = getColinearAlpha(a, b, point);
	//F32 pointOnAB_Y = a.d[1] + ((b.d[1] - a.d[1]) * abAlpha);
	//return _(pointOnAB_Y F32_GREATEQL point.d[1]);
}

static
PixErr isPointInFace(
	const PixalcFPtrs *pAlloc,
	I8Arr *pHandBuf,
	FaceIntern *pFace,
	V3_F32 point,
	bool *pIn
) {
	PixErr err = PIX_ERR_SUCCESS;
	V2_F32 pointV2 = *(V2_F32 *)&point;
	V2_F32 rayB = {.d = {point.d[0], point.d[1] + 1.0f}};
	V2_F32 rayNormal = pixmV2F32LineNormal(_(rayB V2SUB pointV2));
	I32 windNum = 0;
	FaceIter iter = {0};
	faceIterInit(pFace, NULL, NULL, NULL, true, &iter);
	for (; !faceIterSetCorner(&iter); faceIterIncBoundary(&iter)) {
		Corner *pCorner = iter.pCorner;
		I32 size = pFace->pRoots[iter.boundary].originSize;
		PIXALC_DYN_ARR_RESIZE(I8, pAlloc, pHandBuf, size);
		I32 i = 0;
		do {
			PIX_ERR_RETURN_IFNOT_COND(err, i < size, "infinite or astray loop");
			F32 sign = _(_(*(V2_F32 *)&pCorner->pos V2SUB pointV2) V2DOT rayNormal);
			pHandBuf->pArr[i] = _(sign F32_EQL .0f) ? HAND_ON :
				_(sign F32_GREAT .0f) ? HAND_RIGHT : HAND_LEFT; 
		} while(++i, pCorner = pCorner->pNextOrigin, pCorner != iter.pStart);
		typedef struct OverlapChain {
			Hand hand;
			bool active;
			bool onRay;
		} OverlapChain;
		OverlapChain chain = {0};
		Corner *pStart = NULL;
		I32 iStart = 0;
		do {
			PIX_ERR_RETURN_IFNOT_COND(err, iStart < size, "infinite or astray loop");
			if (pHandBuf->pArr[iStart] != HAND_ON) {
				pStart = pCorner;
				break;
			}
		} while(++iStart, pCorner = pCorner->pNextOrigin, pCorner != iter.pStart);
		if (!pStart) {
			continue;//face and ray are colinear - skip this boundary
		}
		i = 0;
		do {
			PIX_ERR_RETURN_IFNOT_COND(err, i < size, "infinite or astray loop");
			I32 iOffset = (iStart + i) % size;
			I32 iNext = (iOffset + 1) % size;
			I32 iPrev = iOffset ? iOffset - 1 : size - 1;
			bool nextIsOn = pHandBuf->pArr[iNext] == HAND_ON;
			if (pHandBuf->pArr[iOffset] != HAND_ON) {
				if (!nextIsOn && pHandBuf->pArr[iOffset] != pHandBuf->pArr[iNext] &&
					(
						_(pCorner->pos.d[1] F32_GREATEQL pointV2.d[1]) ||
						_(pCorner->pNextOrigin->pos.d[1] F32_GREATEQL pointV2.d[1])
					) &&
					doesIntersectRay(
						pointV2, rayB,
						*(V2_F32 *)&pCorner->pos, *(V2_F32 *)&pCorner->pNextOrigin->pos
				)) {
					++windNum;
				}
				continue;
			}
			bool prevIsOn = pHandBuf->pArr[iPrev] == HAND_ON;
			if (prevIsOn && nextIsOn) {
				continue;
			}
			bool onRay = _(pCorner->pos.d[1] F32_GREATEQL pointV2.d[1]);
			if (!prevIsOn && !nextIsOn) {
				if (onRay && pHandBuf->pArr[iPrev] != pHandBuf->pArr[iNext]) {
					++windNum;
				}
				//else below point or bounce
				continue;
			}
			I32 notOn = prevIsOn ? iNext : iPrev;
			if (chain.active) {
				if (onRay && chain.onRay && chain.hand != pHandBuf->pArr[notOn]) {
					//delayed crossing
					++windNum;
				}
				//else delayed bounce, or delayed crossing that's not above point
				chain.active = false;
			}
			else {
				chain.active = true;
				chain.hand = pHandBuf->pArr[notOn];
				chain.onRay = onRay;
			}
		} while(++i, pCorner = pCorner->pNextOrigin, pCorner != pStart);
	}
	err = faceIterGetErr(&iter);
	PIX_ERR_RETURN_IFNOT(err, "")
	*pIn = windNum % 2;
	return err;
}

		//getIntersectAlpha snaps to 0 or 1 if within threshold, so not using epsilon

typedef struct LabelIterArgs {
	const PixalcFPtrs *pAlloc;
	I8Arr *pHandBuf;
	PixalcLinAlloc *pCornerAlloc;
	FaceIntern *pFaceB;
	bool *pIn;
	bool *pCommonEdges;
} LabelIterArgs;

static
PixErr labelIterStartPredicate(void *pUserData, const Corner *pCorner, bool *pValid) {
	PixErr err = PIX_ERR_SUCCESS;
	if (pCorner->pLink) {
		*pValid = false;
		return err;
	}
	*pValid = true;
	LabelIterArgs *pArgs = pUserData;
	err = isPointInFace(
		pArgs->pAlloc,
		pArgs->pHandBuf,
		pArgs->pFaceB,
		pCorner->pos,
		pArgs->pIn
	);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
bool isEdgeCommon(const Corner *pA) {
	const Corner *pB = pA->pNext->pLink;
	return pA->pLink->pNext == pB || pA->pLink->pPrev == pB;
}

static
PixErr labelIterHandleNoStart(void *pUserData, FaceRootIntern *pRoot, Corner **ppStart) {
	PixErr err = PIX_ERR_SUCCESS;
	LabelIterArgs *pArgs = pUserData;
	bool uncommonEdge = false;
	Corner *pCorner = pRoot->pRoot;
	for (I32 i = 0; !i || pCorner != pRoot->pRoot; ++i, pCorner = pCorner->pNext) {
		PIX_ERR_RETURN_IFNOT_COND(err, i < pRoot->size, "infinite or astray loop");
		PIX_ERR_ASSERT(
			"found an original corner, why was this func called?",
			pCorner->pLink
		);
		if (!isEdgeCommon(pCorner)) {
			uncommonEdge = true;
			break;
		}
	}
	if (!uncommonEdge) {
		*ppStart = NULL;
		*pArgs->pCommonEdges = true;
		return err;
	}
	Corner *pA = pCorner;
	Corner *pB = pCorner->pNext;
	V3_F32 midPoint = _(pA->pos V3ADD _(_(pB->pos V3SUB pA->pos) V3DIVS 2.0f));
	Corner *pNew = NULL;
	pixalcLinAlloc(pArgs->pCornerAlloc, &pNew, 1);
	pNew->pos = midPoint;
	pNew->dontAdd = true;
	insertCorner(pRoot, pA, pNew, true);
	*ppStart = pNew;
	return err;
}

static
bool isCornerDelayed(const Corner *pCorner) {
	return
		pCorner->label == LABEL_CROSS_DELAYED || pCorner->label == LABEL_BOUNCE_DELAYED;

}

static
PixErr labelCrossDir(
	const PixalcFPtrs *pAlloc,
	I8Arr *pHandBuf,
	PixalcLinAlloc *pCornerAlloc,
	FaceIntern *pFaceA, FaceIntern *pFaceB
) {
	PixErr err = PIX_ERR_SUCCESS;
	Corner *pStart = NULL;
	bool in = false;
	bool commonEdges = false;
	LabelIterArgs labelIterArgs = {
		.pAlloc = pAlloc,
		.pHandBuf = pHandBuf,
		.pCornerAlloc = pCornerAlloc,
		.pFaceB = pFaceB,
		.pIn = &in,
		.pCommonEdges = &commonEdges
	};
	FaceIter iter = {0};
	faceIterInit(
		pFaceA,
		&labelIterArgs, labelIterStartPredicate, labelIterHandleNoStart,
		false,
		&iter
	);
	bool chainActive = false;
	CrossDir chainTravel = 0;
	//TODO handle no start corner
	for (; !faceIterSetCorner(&iter); faceIterInc(&iter)) {
		if (!iter.pCorner) {
			//skip this boundary
			if (commonEdges) {
				//TODO handle this case
				commonEdges = false;
			}
			continue;
		}
		if (!iter.pCorner->pLink) {
			continue;
		}
		bool cross = false;
		if (isCornerDelayed(iter.pCorner)) {
			if (!chainActive) {
				chainTravel = in ? CROSS_EXIT : CROSS_ENTRY;
			}
			switch (iter.pCorner->label) {
				case LABEL_BOUNCE_DELAYED:
					if (chainTravel == CROSS_EXIT) {
						iter.pCorner->label = LABEL_CROSS_CANDIDATE;
					}
					cross = true;
					break;
				case LABEL_CROSS_DELAYED:
					iter.pCorner->travel = chainTravel;
					if (chainActive && chainTravel == CROSS_ENTRY ||
						!chainActive && chainTravel == CROSS_EXIT
					) {
						iter.pCorner->cross = true;
					}
					break;
				default:
					PIX_ERR_ASSERT("", false);
			}
			chainActive = !chainActive;
		}
		if (iter.pCorner->cross || cross) {
			iter.pCorner->travel = in ? CROSS_EXIT : CROSS_ENTRY;
			in = !in;
		} 
	}
	err = faceIterGetErr(&iter);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
PixErr getFirstClipEntry(FaceIter *pIter, Corner **ppStart) {
	PixErr err = PIX_ERR_SUCCESS;
	for (; !faceIterSetCorner(pIter); faceIterInc(pIter)) {
		if (!pIter->pCorner->checked &&
			pIter->pCorner->pLink &&
			pIter->pCorner->cross
		) {
			*ppStart = pIter->pCorner;
			faceIterInc(pIter);
			return err;
		}
	}
	err = faceIterGetErr(pIter);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

static
PlycutCornerIdx makeClipCornerIdx(const Corner *pCorner) {
	return (PlycutCornerIdx) {
		.corner = pCorner->originCorner,
		.boundary = pCorner->boundary
	};
}

static
void setOutCornerInfo(PlycutCorner *pOut, const Corner *pCorner) {
	bool inClip = pCorner->face == PCUT_FACE_CLIP;
	if (!pCorner->pLink) {
		pOut->type = inClip ? PLYCUT_ORIGIN_CLIP : PLYCUT_ORIGIN_SUBJECT;
		pOut->info.origin.corner = makeClipCornerIdx(pCorner);
		return;
	}
	const Corner *pClip = inClip ? pCorner : pCorner->pLink;
	const Corner *pSubj = inClip ? pCorner->pLink : pCorner;
	switch (pCorner->label) {
		case LABEL_CROSS:
			pOut->type = PLYCUT_INTERSECT;
			pOut->info.intersect.clipCorner = makeClipCornerIdx(pClip);
			pOut->info.intersect.subjCorner = makeClipCornerIdx(pSubj);
			pOut->info.intersect.clipAlpha = pClip->alpha;
			pOut->info.intersect.subjAlpha = pSubj->alpha;
			break;
		case LABEL_ON_ON:
		case LABEL_CROSS_DELAYED:
		case LABEL_CROSS_CANDIDATE:
		case LABEL_BOUNCE:
		case LABEL_BOUNCE_DELAYED:
			if (pCorner->original && pCorner->pLink->original) {
				pOut->type = PLYCUT_ON_VERT;
				pOut->info.onVert.clipCorner = makeClipCornerIdx(pClip);
				pOut->info.onVert.subjCorner = makeClipCornerIdx(pSubj);
			}
			else {
				const Corner *pVert = pCorner->original ? pCorner : pCorner->pLink;
				pOut->type = pVert->face == PCUT_FACE_CLIP ?
					PLYCUT_ON_SUBJECT_EDGE : PLYCUT_ON_CLIP_EDGE;
				pOut->info.onEdge.vertCorner = makeClipCornerIdx(pVert);
				pOut->info.onEdge.edgeCorner = makeClipCornerIdx(pVert->pLink);
				pOut->info.onEdge.alpha = pVert->pLink->alpha;
			}
			break;
		default:
			PIX_ERR_ASSERT("invalid label for this phase", false);
	}
}

static
void addCorner(PlycutFaceArr *pArr, I32 face, const Corner *pCorner) {
	if (pCorner->dontAdd) {
		return;
	}
	PlycutCorner *pNew = NULL;
	pixalcLinAlloc(&pArr->cornerAlloc, &pNew, 1);
	pNew->pos = pCorner->pos;
	if (!pArr->pArr[face].pRoot) {
		pArr->pArr[face].pRoot = pNew;
	}
	else {
		PlycutCorner *pOut = pArr->pArr[face].pRoot;
		while (pOut->pNext) {
			pOut = pOut->pNext;
		}
		pOut->pNext = pNew;
		pNew->pPrev = pOut;
	}
	setOutCornerInfo(pNew, pCorner);
	++pArr->pArr[face].size;
}

static
I32 beginFace(
	const PixalcFPtrs *pAlloc,
	PlycutFaceArr *pArr,
	const Corner *pStart
) {
	I32 newIdx = -1;
	PIXALC_DYN_ARR_ADD(PlycutFaceRoot, pAlloc, pArr, newIdx);
	pArr->pArr[newIdx] = (PlycutFaceRoot) {0};
	return newIdx;
}

static
void reverseWind(PlycutFaceArr *pArr, I32 face) {
	PlycutCorner *pCorner = pArr->pArr[face].pRoot;
	if (!pCorner) {
		return;
	}
	do {
		PlycutCorner *pNext = pCorner->pNext;
		pCorner->pNext = pCorner->pPrev;
		pCorner->pPrev = pNext;
		if (!pNext) {
			break;
		}
		pCorner = pNext;
	} while(true);
	pArr->pArr[face].pRoot = pCorner;
}

static
PixErr makeClippedFaces(
	const PixalcFPtrs *pAlloc,
	FaceIntern *pClipFace,
	FaceIntern *pSubjFace,
	PlycutFaceArr *pOutArr
) {
	PixErr err = PIX_ERR_SUCCESS;
	I32 clipSize = getFaceSize(pClipFace);
	I32 subjSize = getFaceSize(pSubjFace);
	FaceIter iter = {0};
	faceIterInit(pSubjFace, NULL, NULL, NULL, false, &iter);
	do {
		Corner *pStart = NULL;
		err = getFirstClipEntry(&iter, &pStart);
		PIX_ERR_RETURN_IFNOT(err, "");
		/*
		if (!pStart && iter.pFace != pClipFace) {
			PIX_ERR_ASSERT("", iter.pFace == pSubjFace && faceIterAtEnd(&iter));
			faceIterInit(pClipFace, NULL, NULL, NULL, false, &iter);
			err = getFirstClipEntry(&iter, &pStart);
			STUC_RETURN_ERR_IFNOT(err, "");
		}
		*/
		if (!pStart) {
			PIX_ERR_ASSERT("", faceIterAtEnd(&iter));
			return err;
		}
		I32 outFace = beginFace(pAlloc, pOutArr, pStart);
		Corner *pCorner = pStart;
		I32 i = 0;
		do {
			PIX_ERR_RETURN_IFNOT_COND(
				err,
				i < clipSize + subjSize,
				"infinite or astray loop"
			);
			pCorner->checked = true;
			CrossDir travel = pCorner->travel;
			do {
				pCorner = travel == CROSS_ENTRY ? pCorner->pNext : pCorner->pPrev;
				pCorner->checked = true;
				addCorner(pOutArr, outFace, pCorner);
			} while((!pCorner->cross || pCorner->travel == travel) && pCorner != pStart);
			PIX_ERR_ASSERT("", pCorner->pLink);
			if (pCorner != pStart) {
				pCorner = pCorner->pLink;
			}
		} while(++i, pCorner != pStart);
		if (pStart->travel == CROSS_EXIT) {
			reverseWind(pOutArr, outFace);
		}
		//unlike clip and subject, out-face corners are not circularly linked
	} while(true);
	return err;
}

static
I32 getFaceInputSize(PlycutInput face) {
	I32 size = 0;
	for (I32 i = 0; i < face.boundaries; ++i) {
		size += face.pSizes[i];
	}
	return size;
}

static
I32 getFaceSize(const FaceIntern *pFace) {
	I32 size = 0;
	for (I32 i = 0; i < pFace->boundaries; ++i) {
		size += pFace->pRoots[i].size;
	}
	return size;
}

static
PixErr processCandidates(FaceIntern *pSubj) {
	PixErr err = PIX_ERR_SUCCESS;
	FaceIter iter = {0};
	faceIterInit(pSubj, NULL, NULL, NULL, false, &iter);
	for (; !faceIterSetCorner(&iter); faceIterInc(&iter)) {
		if (iter.pCorner->label == LABEL_CROSS_CANDIDATE &&
			iter.pCorner->pLink->label == LABEL_CROSS_CANDIDATE
		) {
			setLinkCross(iter.pCorner);
		}
	}
	err = faceIterGetErr(&iter);
	PIX_ERR_RETURN_IFNOT(err, "");
	return err;
}

plycutClipIntern(
	const PixalcFPtrs *pAlloc,
	PixalcLinAlloc *pCornerAlloc,
	I32 initSize,
	FaceIntern *pClip, FaceIntern *pSubj,
	PlycutFaceArr *pOut
) {
	PixErr err = PIX_ERR_SUCCESS;
	//find intersections
	FaceIter clipIter = {0};
	faceIterInit(pClip, NULL, NULL, NULL, true, &clipIter);
	for (; !faceIterSetCorner(&clipIter); faceIterInc(&clipIter)) {
		FaceIter subjIter = {0};
		faceIterInit(pSubj, NULL, NULL, NULL, true, &subjIter);
		for (; !faceIterSetCorner(&subjIter); faceIterInc(&subjIter)) {
			err = intersectHalfEdges(
				pCornerAlloc,
				faceIterGetRoot(&clipIter), faceIterGetRoot(&subjIter),
				clipIter.pCorner, subjIter.pCorner 
			);
			PIX_ERR_THROW_IFNOT(err, "", 0);
		}
		err = faceIterGetErr(&subjIter);
		PIX_ERR_RETURN_IFNOT(err, "");
	}
	err = faceIterGetErr(&clipIter);
	PIX_ERR_RETURN_IFNOT(err, "");

	err = labelCrossOrBounce(pSubj);
	PIX_ERR_THROW_IFNOT(err, "", 0);
	{
		I8Arr handBuf = {0};
		err = labelCrossDir(pAlloc, &handBuf, pCornerAlloc, pSubj, pClip);
		PIX_ERR_THROW_IFNOT(err, "", 1);
		err = labelCrossDir(pAlloc, &handBuf, pCornerAlloc, pClip, pSubj);
		PIX_ERR_THROW_IFNOT(err, "", 1);
		PIX_ERR_CATCH(1, err, ;)
		if (handBuf.pArr) {
			pAlloc->fpFree(handBuf.pArr);
		}
		PIX_ERR_THROW_IFNOT(err, "", 0);
	}

	err = processCandidates(pSubj);
	PIX_ERR_THROW_IFNOT(err, "", 0);

	*pOut = (PlycutFaceArr) {0};
	pixalcLinAllocInit(pAlloc, &pOut->cornerAlloc, sizeof(PlycutCorner), initSize, true);
	err = makeClippedFaces(pAlloc, pClip, pSubj, pOut);
	PIX_ERR_THROW_IFNOT(err, "", 0);
	PIX_ERR_CATCH(0, err, ;);
	return err;
}

plycutClipInitMem(
	const PixalcFPtrs *pAlloc,
	PlycutInput clipInput, PlycutInput subjInput,
	PixalcLinAlloc *pRootAlloc,
	PixalcLinAlloc *pCornerAlloc,
	I32 *pInitSize
) {
	I32 rootCount = clipInput.boundaries + subjInput.boundaries;
	pixalcLinAllocInit(pAlloc, pRootAlloc, sizeof(FaceRootIntern), rootCount, true);
	*pInitSize = getFaceInputSize(clipInput) + getFaceInputSize(subjInput);
	pixalcLinAllocInit(pAlloc, pCornerAlloc, sizeof(Corner), *pInitSize, true);
}

plycutClipInitCorner(
	FaceIntern *pFace,
	I32 boundary,
	I32 corner,
	PlycutClipOrSubj face,
	V3_F32 pos
) {
	I32 jNext = (corner + 1) % pFace->pRoots[boundary].size;
	I32 jPrev = corner ? corner - 1 : pFace->pRoots[boundary].size - 1;
	Corner *pCorner = pFace->pRoots[boundary].pRoot + corner;
	pCorner->pNext = pFace->pRoots[boundary].pRoot + jNext;
	pCorner->pPrev = pFace->pRoots[boundary].pRoot + jPrev;
	pCorner->pNextOrigin = pCorner->pNext;
	pCorner->pPrevOrigin = pCorner->pPrev;
	pCorner->boundary = boundary;
	pCorner->originCorner = corner;
	pCorner->original = true;
	pCorner->face = face;
	pCorner->pos = pos;
}

void plycutFaceArrDestroy(const PixalcFPtrs *pAlloc, PlycutFaceArr *pArr) {
	PIX_ERR_ASSERT("", pArr->count <= pArr->size && pArr->count >= 0);
	if (pArr->cornerAlloc.valid) {
		pixalcLinAllocDestroy(&pArr->cornerAlloc);
	}
	if (pArr->pArr) {
		pAlloc->fpFree(pArr->pArr);
	}
	*pArr = (PlycutFaceArr) {0};
}
