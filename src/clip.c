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
	CROSS_ENTRY,
	CROSS_EXIT
} CrossDir;

typedef enum Face {
	FACE_CLIP,
	FACE_SUBJECT
} Face;

typedef struct Corner {
	struct Corner *pNext;
	struct Corner *pPrev;
	struct Corner *pLink;//using 'link' to refer to an intersect or overlap point
	V3_F32 pos;
	F32 t;
	Label label;
	CrossDir travel;
	Face face;
	bool intersect;
} Corner;

static
void cornerListInit(
	LinAlloc *pAlloc,
	const void *pUserData,
	const Mesh *pMesh, const FaceRange *pFace,
	V3_F32 (* getPos)(const void *, const Mesh *, const FaceRange *, I32),
	Corner **ppRoot,
	Face face
) {
	stucLinAlloc(pAlloc, ppRoot, pFace->size);
	for (I32 i = 0; i < pFace->size; ++i) {
		Corner *pCorner = (*ppRoot) + i;
		I32 iNext = (i + 1) % pFace->size;
		pCorner->pNext = (*ppRoot) + iNext;
		pCorner->face = face;
		pCorner->pos = getPos(pUserData, pMesh, pFace, i);
	}
}

static
getSignedArea(V2_F32 a, V2_F32 b, V2_F32 c) {
	return _(_(b V2SUB a) V2CROSS _(c V2SUB a));
}

//returns 1 if edges are parallel
static
I32 getIntersectAlpha(V3_F32 a, V3_F32 b, V3_F32 c, V3_F32 d, F32 *pAlpha) {
	F32 acd = getSignedArea(*(V2_F32 *)&a, *(V2_F32 *)&c, *(V2_F32 *)&d);
	F32 bcd = getSignedArea(*(V2_F32 *)&b, *(V2_F32 *)&c, *(V2_F32 *)&d);
	F32 divisor = (acd - bcd);
	if (_(divisor F32_EQL .0f)) {
		return 1;
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
Result insertCorner(Corner *pCorner, Corner *pNew) {
	Result err = STUC_SUCCESS;
	while (!pCorner->pNext->pLink && _(pNew->t F32_GREAT pCorner->t)) {
		STUC_RETURN_ERR_IFNOT_COND(err, _(pNew->t F32_NOTEQL pCorner->t), "degen verts");
		pCorner = pCorner->pNext;
	}
	pNew->face = pCorner->face;
	pNew->pNext = pCorner->pNext;
	pCorner->pNext = pNew;
	pNew->pNext->pPrev = pNew;
	pNew->pPrev = pCorner;
	return err;
}

static
Result insertT(LinAlloc *pAlloc, Corner *pEdge, F32 aEdge, Corner *pPoint) {
	Result err = STUC_SUCCESS;
	Corner *pCopy = NULL;
	stucLinAlloc(pAlloc, &pCopy, 1);
	*pCopy = *pPoint;
	pCopy->t = aEdge;
	err = insertCorner(pEdge, pCopy);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result handleXIntersect(LinAlloc *pAlloc, Corner *pClip, Corner *pSubj, F32 tSubjEdge) {
	Result err = STUC_SUCCESS;
	Corner *pIntersect = NULL;
	stucLinAlloc(pAlloc, &pIntersect, 2);//2 copies for each list
	//subject face is 3D (clip face is 2D), so we use it to calc interesction
	pIntersect->pos =
		_(pSubj->pos V3ADD _(_(pSubj->pNext->pos V3SUB pSubj->pos) V3MULS tSubjEdge));
	pIntersect->intersect = true;
	pIntersect[1] = *pIntersect;//copy to link
	err = insertCorner(pClip, pIntersect);
	STUC_RETURN_ERR_IFNOT(err, "");
	err = insertCorner(pSubj, pIntersect + 1);
	STUC_RETURN_ERR_IFNOT(err, "");
	return err;
}

static
Result insertIntersect(
	LinAlloc *pAlloc,
	Corner *pClip, Corner *pSubj,
	bool *pColinear
) {
	Result err = STUC_SUCCESS;
	F32 aClipEdge = .0f;
	F32 aSubjEdge = .0f;
	if (getIntersectAlpha(pSubj->pos, pSubj->pNext->pos, pClip->pos, pClip->pNext->pos, &aClipEdge) ||
		getIntersectAlpha(pClip->pos, pClip->pNext->pos, pSubj->pos, pSubj->pNext->pos, &aSubjEdge)
	) {
		*pColinear = true;
		return err;
	}
	else {
		*pColinear = false;
	}
	if (_(aSubjEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f)) {
		//X intersection
		err = handleXIntersect(pAlloc, pClip, pSubj, aSubjEdge);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (
		_(aSubjEdge F32_EQL .0f) &&
		_(aClipEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f)
	) {
		//T intersection on clip edge
		err = insertT(pAlloc, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (
		_(aClipEdge F32_EQL .0f) &&
		_(aSubjEdge F32_GREAT .0f) && _(aSubjEdge F32_LESS 1.0f)
	) {
		//T intersection on subject edge
		err = insertT(pAlloc, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aClipEdge F32_EQL aSubjEdge) && _(aClipEdge F32_EQL .0f)) {
		//V intersection
		STUC_RETURN_ERR_IFNOT_COND(err, !pClip->pLink && !pSubj->pLink, "degen verts");
		pClip->pLink = pSubj;
		pSubj->pLink = pClip;
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
Result insertOverlap(LinAlloc *pAlloc, Corner *pClip, Corner *pSubj) {
	Result err = STUC_SUCCESS;
	F32 aClipEdge = getColinearAlpha(
		*(V2_F32 *)&pClip->pos, *(V2_F32 *)&pClip->pNext->pos,
		*(V2_F32 *)&pSubj->pos
	);
	F32 aSubjEdge = getColinearAlpha(
		*(V2_F32 *)&pSubj->pos, *(V2_F32 *)&pSubj->pNext->pos,
		*(V2_F32 *)&pClip->pos
	);
	if (_(aSubjEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f)) {
		//X overlap
		err = insertT(pAlloc, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
		err = insertT(pAlloc, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (
		(_(aSubjEdge F32_LESS .0f) || _(aSubjEdge F32_GREATEQL 1.0f)) &&
		_(aClipEdge F32_GREAT .0f) && _(aClipEdge F32_LESS 1.0f)
	) {
		//T overlap on clip edge
		err = insertT(pAlloc, pClip, aClipEdge, pSubj);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (
		(_(aClipEdge F32_LESS .0f) || _(aClipEdge F32_GREATEQL 1.0f)) &&
		_(aSubjEdge F32_GREAT .0f) && _(aSubjEdge F32_LESS 1.0f)
	) {
		//T overlap on subj edge
		err = insertT(pAlloc, pSubj, aSubjEdge, pClip);
		STUC_RETURN_ERR_IFNOT(err, "");
	}
	else if (_(aSubjEdge F32_EQL aClipEdge) && _(aSubjEdge F32_EQL .0f)) {
		//V overlap
		pClip->pLink = pSubj;
		pSubj->pLink = pClip;
	}
	return err;
}

static
Result intersectHalfEdges(LinAlloc *pAlloc, Corner *pClip, Corner *pSubj) {
	Result err = STUC_SUCCESS;
	STUC_RETURN_ERR_IFNOT_COND(
		err,
		_(*(V2_F32 *)&pClip->pos V2NOTEQL * (V2_F32 *)&pClip->pNext->pos) &&
		_(*(V2_F32 *)&pSubj->pos V2NOTEQL * (V2_F32 *)&pSubj->pNext->pos),
		"degen edge(s)"
	);
	bool colinear = false;
	err = insertIntersect(pAlloc, pClip, pSubj, &colinear);
	STUC_RETURN_ERR_IFNOT(err, "");
	if (colinear) {
		//colinear
		insertOverlap(pAlloc, pClip, pSubj);
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
void labelCrossOrBounce(Corner *pSubjRoot) {
	Corner *pSubj = pSubjRoot;
	do {
		if (!pSubj->pLink) {
			continue;
		}
		Corner *pClip = pSubj->pLink;
		LocalInfo info = getLocalInfoForIntersect(pClip, pSubj);
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
		setLinkLabel(pSubj, label);
	} while(pSubj = pSubj->pNext, pSubj != pSubjRoot);

	typedef struct OverlapChain {
		Corner *pStart;
	} OverlapChain;
	OverlapChain chain = {0};
	do {
		Label label = pSubj->label;
		if (label == LABEL_CROSS || label == LABEL_BOUNCE) {
			STUC_ASSERT("", !chain.pStart);
			continue;
		}
		if (!chain.pStart) {
			STUC_ASSERT("", label == LABEL_LEFT_ON || label == LABEL_RIGHT_ON);
			chain.pStart = pSubj;
		}
		else if (label == LABEL_ON_ON){
			setLinkLabel(pSubj, LABEL_BOUNCE_DELAYED);
		}
		else {
			STUC_ASSERT("", label == LABEL_ON_LEFT || label == LABEL_ON_RIGHT);
			label = label == chain.pStart->label ?
				LABEL_BOUNCE_DELAYED : LABEL_CROSS_DELAYED;
			setLinkLabel(pSubj, label);
			setLinkLabel(chain.pStart, label);
			chain.pStart = NULL;
		}
	} while(pSubj = pSubj->pNext, pSubj != pSubjRoot);
}

static
Result labelCrossDir(const Corner *pSubjRoot) {
	Result err = STUC_SUCCESS;
	//implement
	return err;
}

Result stucClip(
	const StucAlloc *pAlloc,
	const void *pUserData,
	const Mesh *pClipMesh, const FaceRange *pClip,
	V3_F32 (* clipGetPos)(const void *, const Mesh *, const FaceRange *, I32),
	const Mesh *pSubjMesh, const FaceRange *pSubj,
	V3_F32 (* subjGetPos)(const void *, const Mesh *, const FaceRange *, I32)
) {
	//subject abbreviated to subj
	Result err = STUC_SUCCESS;
	LinAlloc cornerAlloc = {0};
	{
		I32 initSize = pClip->size + pSubj->size;
		stucLinAllocInit(pAlloc, &cornerAlloc, sizeof(Corner), initSize, true);
	}
	Corner *pClipRoot = NULL;
	Corner *pSubjRoot = NULL;
	cornerListInit(
		&cornerAlloc, pUserData,
		pClipMesh, pClip, clipGetPos,
		&pClipRoot,
		FACE_CLIP
	);
	cornerListInit(
		&cornerAlloc,
		pUserData,
		pSubjMesh, pSubj, subjGetPos,
		&pSubjRoot,
		FACE_SUBJECT
	);
	//find intersections
	for (I32 i = 0; i < pClip->size; ++i) {
		Corner *pClipCorner = pClipRoot + i;
		for (I32 j = 0; j < pSubj->size; ++j) {
			Corner *pSubjCorner = pSubjRoot + j;
			err = intersectHalfEdges(&cornerAlloc, pClipCorner, pSubjCorner);
			STUC_THROW_IFNOT(err, "", 0);
		}
	}
	labelCrossOrBounce(pSubjRoot);
	labelCrossDir(pSubjRoot);
	STUC_CATCH(0, err, ;);
	stucLinAllocDestroy(&cornerAlloc);
	return err;
}

/*
notes:
	- only begin tracing a new face on entry points.
		Starting on an exit will reverse the face orientation
*/
