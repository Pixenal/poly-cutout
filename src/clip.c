#include <float.h>
#include <math.h>

#include <clip.h>
#include <utils.h>

typedef enum Label {
	LABEL_NONE,
	LABEL_CROSS,
	LABEL_CROSS_DELAYED,
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

typedef enum ClipOrSubj {
	FACE_CLIP,
	FACE_SUBJECT
} ClipOrSubj;

typedef struct Corner {
	struct Corner *pNext;
	struct Corner *pPrev;
	struct Corner *pNextOrigin;
	struct Corner *pPrevOrigin;
	struct Corner *pLink;//using 'link' to refer to an intersect or overlap point
	V3_F32 pos;
	I32 originCorner;
	I32 boundary;
	F32 alpha;
	Label label;
	CrossDir travel;
	ClipOrSubj face;
	bool checked;
	bool original;
} Corner;

typedef struct Funcs {
	V2_F32 (* getClipPos)(const void *, const void *, I32, I32);
	V3_F32 (* getSubjPos)(const void *, const void *, I32, I32);
} Funcs;

typedef struct FaceRoot {
	Corner *pRoot;
	I32 size;
	I32 originSize;
	I32 boundary;
} FaceRoot;

typedef struct Face {
	FaceRoot *pRoots;
	I32 boundaries;
} Face;

typedef struct FaceIter {
	void *pUserData;
	Face *pFace;
	Corner *pCorner;
	Corner *pStart;
	Result (* fpStartPredicate)(void *, const Corner *, bool *);
	Result (* fpHandleNoStart)(void *, FaceRoot *, Corner **);
	I32 boundary;
	I32 corner;
	I32 count;
	Result err;
	bool originOnly;
	bool skipBoundary;
} FaceIter;

static
void faceIterInit(
	Face *pFace,
	void *pUserData,
	Result (* fpStartPredicate)(void *, const Corner *, bool *),
	Result (* fpHandleNoStart)(void *, FaceRoot *, Corner **),
	bool originOnly,
	FaceIter *pIter
	) {
	*pIter = (FaceIter) {
		.pUserData = pUserData,
		.pFace = pFace,
		.err = STUC_SUCCESS,
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
Result faceIterInternFindValidStart(FaceIter *pIter) {
	Result err = STUC_SUCCESS;
	I32 i = 0;
	bool valid = false;
	I32 size = faceIterInternGetSize(pIter);
	while (
		err = pIter->fpStartPredicate(pIter->pUserData, pIter->pCorner, &valid),
		!valid
	) {
		if (err != STUC_SUCCESS) {
			break;
		}
		if (i >= size) {
			STUC_ASSERT("", i == size);
			STUC_RETURN_ERR_IFNOT_COND(
				err,
				pIter->fpHandleNoStart,
				"no valid start corner found"
			);
			FaceRoot *pRoot = pIter->pFace->pRoots + pIter->boundary;
			err = pIter->fpHandleNoStart(pIter->pUserData, pRoot, &pIter->pCorner);
			STUC_RETURN_ERR_IFNOT(err, "");
			if (pIter->pCorner) {
				err = pIter->fpStartPredicate(pIter->pUserData, pIter->pCorner, &valid);
				STUC_RETURN_ERR_IFNOT(err, "");
				STUC_ASSERT(
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
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
bool faceIterAtEnd(FaceIter *pIter) {
	if (pIter->err != STUC_SUCCESS ||
		pIter->boundary >= pIter->pFace->boundaries ||
		pIter->boundary == pIter->pFace->boundaries - 1 &&
		pIter->corner >= faceIterInternGetSize(pIter)
	) {
		return true;
	}
	if (pIter->corner) {
		faceIterInternIncCorner(pIter);
	}
	else {
		FaceRoot *pRoot = pIter->pFace->pRoots + pIter->boundary;
		pIter->pCorner = pRoot->pRoot;
		if (pIter->fpStartPredicate) {
			pIter->err = faceIterInternFindValidStart(pIter);
			STUC_RETURN_ERR_IFNOT(pIter->err, "");
		}
		pIter->pStart = pIter->pCorner;
	}
	return false;
	STUC_CATCH(0, pIter->err, ;);
	return true;
}

static
void faceIterInc(FaceIter *pIter) {
	STUC_ASSERT("", pIter->boundary < pIter->pFace->boundaries);
	++pIter->corner;
	if (!pIter->pStart || pIter->corner >= faceIterInternGetSize(pIter)) {
		if (!pIter->pStart) {
			STUC_ASSERT("start is null, but corner isn't?", !pIter->pCorner);
		}
		else {
			Corner *pCornerNext = pIter->originOnly ?
				pIter->pCorner->pNextOrigin : pIter->pCorner->pNext;
			STUC_THROW_IFNOT_COND(
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
	STUC_CATCH(0, pIter->err, ;);
	return;
}

static
FaceRoot *faceIterGetRoot(FaceIter *pIter) {
	STUC_ASSERT("", pIter->boundary < pIter->pFace->boundaries);
	return pIter->pFace->pRoots + pIter->boundary;
}

static
Result faceIterGetErr(const FaceIter *pIter) {
	return pIter->err;
}

static
V3_F32 callGetClipPos(
	const void *pUserData,
	const void *pMesh,
	const Funcs *pFuncs,
	I32 boundary,
	I32 corner
) {
	V2_F32 pos = pFuncs->getClipPos(pUserData, pMesh, boundary, corner);
	return (V3_F32) {.d = {pos.d[0], pos.d[1], .0f}};
}

static
V3_F32 callGetSubjPos(
	const void *pUserData,
	const void *pMesh,
	const Funcs *pFuncs,
	I32 boundary,
	I32 corner
) {
	return pFuncs->getSubjPos(pUserData, pMesh, boundary, corner);
}

static
void cornerListInit(
	LinAlloc *pRootAlloc,
	LinAlloc *pCornerAlloc,
	const void *pUserData,
	const void *pMesh, ClipInput inputFace,
	V3_F32 (* getPos)(const void *, const void *, const Funcs *, I32, I32),
	const Funcs *pFuncs,
	Face *pFace,
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
			Corner *pCorner = pFace->pRoots[i].pRoot + j;
			I32 jNext = (j + 1) % pFace->pRoots[i].size;
			I32 jPrev = j ? j - 1 : pFace->pRoots[i].size - 1;
			pCorner->pNext = pFace->pRoots[i].pRoot + jNext;
			pCorner->pPrev = pFace->pRoots[i].pRoot + jPrev;
			pCorner->pNextOrigin = pCorner->pNext;
			pCorner->pPrevOrigin = pCorner->pPrev;
			pCorner->boundary = i;
			pCorner->original = true;
			pCorner->face = face;
			pCorner->pos = getPos(pUserData, pMesh, pFuncs, i, j);
		}
	}
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
	F32 divisor = (acd - bcd);
	if (_(divisor F32_EQL .0f)) {
		return _(acd F32_EQL .0f) ? 2 : 1;
	}
	*pAlpha = acd / divisor;
	return 0;
}

typedef enum Intersect {
	INTERSECT_NONE,
	INTERSECT_CROSS,
	INTERSECT_T,
	INTERSECT_V
} Intersect;

static
Result insertCorner(FaceRoot *pRoot, Corner *pCorner, Corner *pNew, bool makeOriginal) {
	Result err = STUC_SUCCESS;
	if (!makeOriginal) {
		while (
			!pCorner->pNext->original &&
			_(pNew->alpha F32_GREAT pCorner->pNext->alpha)
		) {
			pCorner = pCorner->pNext;
			STUC_RETURN_ERR_IFNOT_COND(
				err,
				_(pNew->alpha F32_NOTEQL pCorner->alpha),
				"degen verts"
			);
		}
	}
	pNew->face = pCorner->face;
	pNew->pNext = pCorner->pNext;
	pCorner->pNext = pNew;
	pNew->pNext->pPrev = pNew;
	pNew->pPrev = pCorner;
	pNew->original = makeOriginal;
	pNew->boundary = pRoot->boundary;
	++pRoot->size;
	return err;
}

static
void linkCorners(Corner *pA, Corner *pB) {
	STUC_ASSERT("trying to link corners(s) with existing link", !pA->pLink && !pB->pLink);
	pA->pLink = pB;
	pB->pLink = pA;
}

static
Result insertT(LinAlloc *pAlloc, FaceRoot *pRoot, Corner *pEdge, F32 aEdge, Corner *pPoint) {
	Result err = STUC_SUCCESS;
	Corner *pCopy = NULL;
	stucLinAlloc(pAlloc, &pCopy, 1);
	*pCopy = *pPoint;
	pCopy->alpha = aEdge;
	linkCorners(pCopy, pPoint);
	err = insertCorner(pRoot, pEdge, pCopy, false);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result handleXIntersect(
	LinAlloc *pAlloc,
	FaceRoot *pClipRoot, FaceRoot *pSubjRoot,
	Corner *pClip, Corner *pSubj,
	F32 aClipEdge, F32 aSubjEdge
) {
	Result err = STUC_SUCCESS;
	Corner *pIntersect = NULL;
	stucLinAlloc(pAlloc, &pIntersect, 2);//2 copies for each list
	//subject face is 3D (clip face is 2D), so we use it to calc interesction
	pIntersect[0].pos = _(pSubj->pos V3ADD _(
		_(pSubj->pNextOrigin->pos V3SUB pSubj->pos) V3MULS aSubjEdge)
	);
	pIntersect[1] = *pIntersect;
	pIntersect[0].alpha = aClipEdge;
	pIntersect[1].alpha = aSubjEdge;
	linkCorners(pIntersect, pIntersect + 1);
	err = insertCorner(pClipRoot, pClip, pIntersect, false);
	STUC_RETURN_ERR_IFNOT(err, "");
	err = insertCorner(pSubjRoot, pSubj, pIntersect + 1, false);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result insertIntersect(
	LinAlloc *pAlloc,
	FaceRoot *pClipRoot, FaceRoot *pSubjRoot,
	Corner *pClip, Corner *pSubj,
	bool *pColinear
) {
	Result err = STUC_SUCCESS;
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
	bool aClipIsIn01 = _(aClipEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f);
	bool aSubjIsIn01 = _(aSubjEdge F32_GREAT .0f) && _(aSubjEdge F32_LESS 1.0f);
	if (aClipIsIn01 && aSubjIsIn01) {
		//X intersection
		err = handleXIntersect(
			pAlloc,
			pClipRoot, pSubjRoot,
			pClip, pSubj,
			aClipEdge, aSubjEdge
		);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aSubjEdge F32_EQL .0f) && aClipIsIn01) {
		//T intersection on clip edge
		err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aClipEdge F32_EQL .0f) && aSubjIsIn01) {
		//T intersection on subject edge
		err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aClipEdge F32_EQL aSubjEdge) && _(aClipEdge F32_EQL .0f)) {
		//V intersection
		STUC_RETURN_ERR_IFNOT_COND(err, !pClip->pLink && !pSubj->pLink, "degen verts");
		linkCorners(pClip, pSubj);
	}
	return err;
}

static
F32 getColinearAlpha(V2_F32 a, V2_F32 b, V2_F32 c) {
	V2_F32 ab = _(b V2SUB a);
	V2_F32 ac = _(c V2SUB a);
	//assuming here that a != b, degen edge check should have caught that prior
	return _(ac V2DOT ab) / _(ab V2DOT ab);
}

static
Result insertOverlap(
	LinAlloc *pAlloc,
	FaceRoot *pClipRoot, FaceRoot *pSubjRoot,
	Corner *pClip, Corner *pSubj
) {
	Result err = STUC_SUCCESS;
	F32 aClipEdge = getColinearAlpha(
		*(V2_F32 *)&pClip->pos, *(V2_F32 *)&pClip->pNextOrigin->pos,
		*(V2_F32 *)&pSubj->pos
	);
	F32 aSubjEdge = getColinearAlpha(
		*(V2_F32 *)&pSubj->pos, *(V2_F32 *)&pSubj->pNextOrigin->pos,
		*(V2_F32 *)&pClip->pos
	);
	bool aClipIsIn01 = _(aClipEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f);
	bool aSubjIsIn01 = _(aSubjEdge F32_GREAT .0f) && _(aSubjEdge F32_LESS 1.0f);
	if (aClipIsIn01 && aSubjIsIn01) {
		//X overlap
		err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
		err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (aClipIsIn01 && (!aSubjIsIn01 || _(aSubjEdge F32_EQL 1.0f))) {
		//T overlap on clip edge
		err = insertT(pAlloc, pClipRoot, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (aSubjIsIn01 && (!aClipIsIn01 || _(aClipEdge F32_EQL 1.0f))) {
		//T overlap on subj edge
		err = insertT(pAlloc, pSubjRoot, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aSubjEdge F32_EQL aClipEdge) && _(aSubjEdge F32_EQL .0f)) {
		//V overlap
		linkCorners(pClip, pSubj);
	}
	return err;
}

static
Result intersectHalfEdges(
	LinAlloc *pAlloc, 
	FaceRoot *pClipRoot, FaceRoot *pSubjRoot,
	Corner *pClip, Corner *pSubj
) {
	Result err = STUC_SUCCESS;
	STUC_RETURN_ERR_IFNOT_COND(
		err,
		_(*(V2_F32 *)&pClip->pos V2NOTEQL * (V2_F32 *)&pClip->pNextOrigin->pos) &&
		_(*(V2_F32 *)&pSubj->pos V2NOTEQL * (V2_F32 *)&pSubj->pNextOrigin->pos),
		"degen edge(s)"
	);
	bool colinear = false;
	err = insertIntersect(pAlloc, pClipRoot, pSubjRoot, pClip, pSubj, &colinear);
	STUC_RETURN_ERR_IFNOT(err, "");
	if (colinear) {
		//colinear
		insertOverlap(pAlloc, pClipRoot, pSubjRoot, pClip, pSubj);
	}
	return err;
}

typedef enum Turn {
	TURNS_STRAIGHT,
	TURNS_LEFT,
	TURNS_RIGHT
} Turn;

typedef enum Neighbour {
	NEIGHBOUR_PREV,
	NEIGHBOUR_NEXT
} Neighbour;

typedef struct LocalInfo {
	Turn turnSNext;
	Turn turnCPrev;
	Turn turnCNext;
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
		.turnSNext = _(signSNext F32_EQL .0f) ? TURNS_STRAIGHT :
			_(signSNext F32_GREAT .0f) ? TURNS_LEFT : TURNS_RIGHT,
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
		case TURNS_STRAIGHT:
		case TURNS_LEFT:
			info.turnCPrev = _(signCPrev_0 F32_LESS .0f) || _(signCPrev_1 F32_LESS .0f) ? 
				TURNS_RIGHT : TURNS_LEFT;
			info.turnCNext = _(signCNext_0 F32_LESS .0f) || _(signCNext_1 F32_LESS .0f) ?
				TURNS_RIGHT : TURNS_LEFT;
			break;
		case TURNS_RIGHT:
			info.turnCPrev = _(signCPrev_0 F32_GREAT .0f) || _(signCPrev_1 F32_GREAT .0f) ? 
				TURNS_LEFT : TURNS_RIGHT;
			info.turnCNext = _(signCNext_0 F32_GREAT .0f) || _(signCNext_1 F32_GREAT .0f) ?
				TURNS_LEFT : TURNS_RIGHT;
	}
	return info;
}

static
void setLinkLabel(Corner *pCorner, Label label) {
	STUC_ASSERT("", pCorner->pLink);
	pCorner->pLink->label = pCorner->label = label;
}

static
Result noOnLeftRightPredicate(void *pUserData, const Corner *pCorner, bool *pValid) {
	*pValid =
		pCorner->label != LABEL_ON_ON &&
		pCorner->label != LABEL_ON_LEFT &&
		pCorner->label != LABEL_ON_RIGHT;
	return STUC_SUCCESS;
}

static
Result labelCrossOrBounce(Face *pSubjFace) {
	Result err = STUC_SUCCESS;
	FaceIter subjIter = {0};
	faceIterInit(pSubjFace, NULL, NULL, NULL, false, &subjIter);
	for (; !faceIterAtEnd(&subjIter); faceIterInc(&subjIter)) {
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
			Turn cTurn = info.sPrevLink == NEIGHBOUR_PREV ?
				info.turnCNext : info.turnCPrev;
			label = cTurn == TURNS_RIGHT ? LABEL_ON_LEFT : LABEL_ON_RIGHT;
		}
		else if (info.sNextOnC) {
			//subject,subject+ overlaps with clip,clip+ or clip-,clip
			Turn cTurn = info.sNextLink == NEIGHBOUR_PREV ?
				info.turnCNext : info.turnCPrev;
			label = cTurn == TURNS_RIGHT ? LABEL_LEFT_ON : LABEL_RIGHT_ON;
		}
		else if (info.turnCPrev == info.turnCNext) {
			label = LABEL_BOUNCE;
		}
		else {
			label = LABEL_CROSS;
		}
		setLinkLabel(subjIter.pCorner, label);
	}
	err = faceIterGetErr(&subjIter);
	STUC_RETURN_ERR_IFNOT(err, "");

	typedef struct OverlapChain {
		Corner *pStart;
	} OverlapChain;
	OverlapChain chain = {0};
	faceIterInit(pSubjFace, NULL, noOnLeftRightPredicate, NULL, false, &subjIter);
	for (; !faceIterAtEnd(&subjIter); faceIterInc(&subjIter)) {
		Label label = subjIter.pCorner->label;
		if (!subjIter.pCorner->pLink || label == LABEL_CROSS || label == LABEL_BOUNCE) {
			STUC_ASSERT("", !chain.pStart);
			continue;
		}
		if (!chain.pStart) {
			STUC_ASSERT("", label == LABEL_LEFT_ON || label == LABEL_RIGHT_ON);
			chain.pStart = subjIter.pCorner;
		}
		else if (label == LABEL_ON_ON){
			setLinkLabel(subjIter.pCorner, LABEL_BOUNCE_DELAYED);
		}
		else {
			STUC_ASSERT("", label == LABEL_ON_LEFT || label == LABEL_ON_RIGHT);
			label = (chain.pStart->label == LABEL_LEFT_ON) == (label == LABEL_ON_LEFT) ?
				LABEL_BOUNCE_DELAYED : LABEL_CROSS_DELAYED;
			setLinkLabel(subjIter.pCorner, label);
			setLinkLabel(chain.pStart, LABEL_BOUNCE_DELAYED);
			chain.pStart = NULL;
		}
	}
	err = faceIterGetErr(&subjIter);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result isPointInFace(Face *pFace, V3_F32 point, bool *pIn) {
	Result err = STUC_SUCCESS;
	typedef struct Delayed {
		bool signFirst;
		bool active;
	} Delayed;
	Delayed delayed = {0};
	V2_F32 pointV2 = *(V2_F32 *)&point;
	V3_F32 rayB = (V3_F32) {.d = {point.d[0], point.d[1] + 1.0f, .0f}};
	I32 windNum = 0;
	FaceIter iter = {0};
	faceIterInit(pFace, NULL, NULL, NULL, true, &iter);
	for (; !faceIterAtEnd(&iter); faceIterInc(&iter)) {
		V3_F32 pos = iter.pCorner->pos;
		V3_F32 posNext = iter.pCorner->pNextOrigin->pos;
		F32 aRay = .0f;
		if (getIntersectAlpha(point, rayB, pos, posNext, &aRay) ||
			_(aRay F32_LESSEQL .0f)
		) {
			//lines are parallel, or intersection is outside of ray
			continue;
		}
		F32 aFaceEdge = .0f;
		if (getIntersectAlpha(pos, posNext, point, rayB, &aFaceEdge)) {
			continue;
		}
		if (_(aFaceEdge F32_GREAT .0f) && _(aFaceEdge F32_LESS 1.0f)) {
			++windNum;
		}
		else if (_(aFaceEdge F32_EQL .0f)) {
			V2_F32 rayNormal = v2F32LineNormal(_(*(V2_F32 *)&rayB V2SUB pointV2));
			const Corner *pNeighbour = delayed.active ?
				iter.pCorner->pNextOrigin : iter.pCorner->pPrevOrigin;
			bool signNeighbour = _(
				_(_(*(V2_F32 *)&pNeighbour->pos V2SUB pointV2) V2DOT rayNormal) F32_GREAT
				.0f
			);
			if (!delayed.active) {
				delayed.active = true;
				delayed.signFirst = signNeighbour;
				continue;
			}
			if (signNeighbour != delayed.signFirst) {
				//delayed crossing
				++windNum;
			}
			//else delayed bounce
			delayed.active = false;
		}
	}
	err = faceIterGetErr(&iter);
	STUC_RETURN_ERR_IFNOT(err, "")
	*pIn = windNum % 2;
	return err;
}

static
bool isCornerCross(const Corner *pCorner) {
	return pCorner->label == LABEL_CROSS || pCorner->label == LABEL_CROSS_DELAYED;
}

typedef struct LabelIterArgs {
	LinAlloc *pCornerAlloc;
	Face *pFaceB;
	bool *pIn;
	bool *pCommonEdges;
} LabelIterArgs;

static
Result labelIterStartPredicate(void *pUserData, const Corner *pCorner, bool *pValid) {
	Result err = STUC_SUCCESS;
	if (pCorner->pLink) {
		*pValid = false;
		return err;
	}
	*pValid = true;
	LabelIterArgs *pArgs = pUserData;
	err = isPointInFace(pArgs->pFaceB, pCorner->pos, pArgs->pIn);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
bool isEdgeCommon(const Corner *pA) {
	const Corner *pB = pA->pNext->pLink;
	return pA->pLink->pNext == pB || pA->pLink->pPrev == pB;
}

static
Result labelIterHandleNoStart(void *pUserData, FaceRoot *pRoot, Corner **ppStart) {
	Result err = STUC_SUCCESS;
	LabelIterArgs *pArgs = pUserData;
	bool uncommonEdge = false;
	Corner *pCorner = pRoot->pRoot;
	for (I32 i = 0; !i || pCorner != pRoot->pRoot; ++i, pCorner = pCorner->pNext) {
		STUC_RETURN_ERR_IFNOT_COND(err, i < pRoot->size, "infinite or astray loop");
		STUC_ASSERT(
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
	stucLinAlloc(pArgs->pCornerAlloc, &pNew, 1);
	pNew->pos = midPoint;
	insertCorner(pRoot, pA, pNew, true);
	*ppStart = pNew;
	return err;
}


static
Result labelCrossDir(LinAlloc *pCornerAlloc, Face *pFaceA, Face *pFaceB) {
	Result err = STUC_SUCCESS;
	Corner *pStart = NULL;
	bool in = false;
	bool commonEdges = false;
	LabelIterArgs labelIterArgs = {
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
	//TODO handle no start corner
	for (; !faceIterAtEnd(&iter); faceIterInc(&iter)) {
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
		if (isCornerCross(iter.pCorner)) {
			iter.pCorner->travel = in ? CROSS_EXIT : CROSS_ENTRY;
			in = !in;
		}
	}
	err = faceIterGetErr(&iter);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result getFirstClipEntry(Face *pFace, Corner **ppStart) {
	Result err = STUC_SUCCESS;
	FaceIter iter = {0};
	faceIterInit(pFace, NULL, NULL, NULL, false, &iter);
	for (; !faceIterAtEnd(&iter); faceIterInc(&iter)) {
		if (!iter.pCorner->checked &&
			iter.pCorner->pLink &&
			isCornerCross(iter.pCorner) &&
			iter.pCorner->travel == CROSS_ENTRY
		) {
			*ppStart = iter.pCorner;
			return err;
		}
	}
	err = faceIterGetErr(&iter);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
void setOutCornerInfo(ClipCorner *pOut, const Corner *pCorner) {
	bool inClip = pCorner->face == FACE_CLIP;
	if (!pCorner->pLink) {
		pOut->type = inClip ? CLIP_ORIGIN_CLIP : CLIP_ORIGIN_SUBJECT;
		pOut->info.origin.corner = pCorner->originCorner;
		return;
	}
	const Corner *pClip = inClip ? pCorner : pCorner->pLink;
	const Corner *pSubj = inClip ? pCorner->pLink : pCorner;
	switch (pCorner->label) {
		case LABEL_CROSS:
			pOut->type = CLIP_INTERSECT;
			pOut->info.intersect.clipCorner = pClip->originCorner;
			pOut->info.intersect.subjCorner = pSubj->originCorner;
			pOut->info.intersect.clipAlpha = pClip->alpha;
			pOut->info.intersect.subjAlpha = pSubj->alpha;
			break;
		case LABEL_CROSS_DELAYED:
		case LABEL_BOUNCE:
		case LABEL_BOUNCE_DELAYED:
			if (pCorner->original && pCorner->pLink->original) {
				pOut->type = CLIP_ON_VERT;
				pOut->info.onVert.clipCorner = pClip->originCorner;
				pOut->info.onVert.subjCorner = pSubj->originCorner;
			}
			else {
				const Corner *pVert = pCorner->original ? pCorner : pCorner->pLink;
				pOut->type = pVert->face == FACE_CLIP ?
					CLIP_ON_SUBJECT_EDGE : CLIP_ON_CLIP_EDGE;
				pOut->info.onEdge.vertCorner = pVert->originCorner;
				pOut->info.onEdge.edgeCorner = pVert->pLink->originCorner;
				pOut->info.onEdge.alpha = pVert->pLink->alpha;
			}
			break;
		default:
			STUC_ASSERT("invalid label for this phase", false);
	}
}

static
void addCorner(ClipFaceArr *pArr, I32 face, const Corner *pCorner) {
	ClipCorner *pNew = NULL;
	stucLinAlloc(&pArr->cornerAlloc, &pNew, 1);
	pNew->pos = pCorner->pos;
	if (!pArr->pArr[face].pRoot) {
		pArr->pArr[face].pRoot = pNew;
	}
	else {
		ClipCorner *pOut = pArr->pArr[face].pRoot;
		while (pOut->pNext) {
			pOut = pOut->pNext;
		}
		pOut->pNext = pNew;
	}
	setOutCornerInfo(pNew, pCorner);
	++pArr->pArr[face].size;
}

static
I32 beginFace(
	const StucAlloc *pAlloc,
	ClipFaceArr *pArr,
	const Corner *pStart
) {
	I32 newIdx = -1;
	//TODO use alloc in STUC_DYN_ARR_ADD instead of basic,
	//then replace this with that
	STUC_ASSERT("", pArr->count <= pArr->size);
	if (!pArr->size) {
		pArr->size = 4;
		pArr->pArr = pAlloc->fpMalloc(pArr->size * sizeof(ClipFaceRoot));
	}
	else if (pArr->count < pArr->size) {
		pArr->size *= 2;
		pArr->pArr = pAlloc->fpRealloc(pArr->pArr, pArr->size * sizeof(ClipFaceRoot));
	}
	newIdx = pArr->count;
	pArr->count++;
	pArr->pArr[newIdx] = (ClipFaceRoot) {0};
	return newIdx;
}

static
Result makeClippedFaces(
	const StucAlloc *pAlloc,
	const Face *pClipFace,
	Face *pSubjFace,
	ClipFaceArr *pOutArr
) {
	Result err = STUC_SUCCESS;
	I32 clipSize = getFaceSize(pClipFace);
	I32 subjSize = getFaceSize(pSubjFace);
	do {
		Corner *pStart = NULL;
		err = getFirstClipEntry(pSubjFace, &pStart);
		STUC_RETURN_ERR_IFNOT(err, "");
		if (!pStart) {
			return err;
		}
		I32 outFace = beginFace(pAlloc, pOutArr, pStart);
		Corner *pCorner = pStart->pLink;
		I32 i = 0;
		CrossDir travel = pCorner->travel == CROSS_ENTRY ? CROSS_EXIT : CROSS_ENTRY;
		do {
			STUC_RETURN_ERR_IFNOT_COND(
				err,
				i < clipSize + subjSize,
				"infinite or astray loop"
			);
			addCorner(pOutArr, outFace, pCorner);
			pCorner->checked = true;
			if (pCorner->pLink && isCornerCross(pCorner) &&
				pCorner->travel != travel
			) {
				STUC_ASSERT("", pCorner->pLink);
				pCorner = pCorner->pLink;
				travel = pCorner->travel;
				pCorner->checked = true;
			}
			pCorner = travel == CROSS_ENTRY ? pCorner->pNext : pCorner->pPrev;
		} while(++i, !pCorner->checked);
		//unlike clip and subject, out-face corners are not circularly linked
	} while(true);
	return err;
}

static
I32 getFaceInputSize(ClipInput face) {
	I32 size = 0;
	for (I32 i = 0; i < face.boundaries; ++i) {
		size += face.pSizes[i];
	}
	return size;
}

static
I32 getFaceSize(const Face *pFace) {
	I32 size = 0;
	for (I32 i = 0; i < pFace->boundaries; ++i) {
		size += pFace->pRoots[i].size;
	}
	return size;
}

Result stucClip(
	const StucAlloc *pAlloc,
	const void *pUserData,
	const void *pClipMesh, ClipInput clipInput,
	V2_F32 (* clipGetPos)(const void *, const void *, I32, I32),
	const void *pSubjMesh, ClipInput subjInput,
	V3_F32 (* subjGetPos)(const void *, const void *, I32, I32),
	ClipFaceArr *pOut
) {
	//subject abbreviated to subj
	Result err = STUC_SUCCESS;
	Funcs funcs = {.getClipPos = clipGetPos, .getSubjPos = subjGetPos};
	LinAlloc rootAlloc = {0};
	LinAlloc cornerAlloc = {0};
	{
		I32 rootCount = clipInput.boundaries + subjInput.boundaries;
		stucLinAllocInit(pAlloc, &rootAlloc, sizeof(FaceRoot), rootCount, true);
	}
	I32 initSize = getFaceInputSize(clipInput) + getFaceInputSize(subjInput);
	stucLinAllocInit(pAlloc, &cornerAlloc, sizeof(Corner), initSize, true);
	Face clip = {0};
	Face subj = {0};
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
	//find intersections
	FaceIter clipIter = {0};
	faceIterInit(&clip, NULL, NULL, NULL, true, &clipIter);
	for (; !faceIterAtEnd(&clipIter); faceIterInc(&clipIter)) {
		FaceIter subjIter = {0};
		faceIterInit(&subj, NULL, NULL, NULL, true, &subjIter);
		for (; !faceIterAtEnd(&subjIter); faceIterInc(&subjIter)) {
			err = intersectHalfEdges(
				&cornerAlloc,
				faceIterGetRoot(&clipIter), faceIterGetRoot(&subjIter),
				clipIter.pCorner, subjIter.pCorner 
			);
			STUC_THROW_IFNOT(err, "", 0);
		}
		err = faceIterGetErr(&subjIter);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	err = faceIterGetErr(&clipIter);
	STUC_RETURN_ERR_IFNOT(err, "");

	err = labelCrossOrBounce(&subj);
	STUC_THROW_IFNOT(err, "", 0);
	err = labelCrossDir(&cornerAlloc, &subj, &clip);
	STUC_THROW_IFNOT(err, "", 0);
	err = labelCrossDir(&cornerAlloc, &clip, &subj);
	STUC_THROW_IFNOT(err, "", 0);

	*pOut = (ClipFaceArr) {0};
	stucLinAllocInit(pAlloc, &pOut->cornerAlloc, sizeof(ClipCorner), initSize, true);
	err = makeClippedFaces(pAlloc, &clip, &subj, pOut);
	STUC_THROW_IFNOT(err, "", 0);
	STUC_CATCH(0, err, ;);
	stucLinAllocDestroy(&rootAlloc);
	stucLinAllocDestroy(&cornerAlloc);
	return err;
}

void stucClipFaceArrDestroy(const StucAlloc *pAlloc, ClipFaceArr *pArr) {
	STUC_ASSERT("", pArr->count <= pArr->size && pArr->count >= 0);
	if (pArr->cornerAlloc.valid) {
		stucLinAllocDestroy(&pArr->cornerAlloc);
	}
	if (pArr->pArr) {
		pAlloc->fpFree(pArr->pArr);
	}
	*pArr = (ClipFaceArr) {0};
}
