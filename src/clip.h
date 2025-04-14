#pragma once

#include <uv_stucco_intern.h>

Result stucClip(
	const StucAlloc *pAlloc,
	const void *pUserData,
	const Mesh *pClipMesh, const FaceRange *pClip,
	V3_F32 (* clipGetPos)(const void *, const Mesh *, const FaceRange *, I32),
	const Mesh *pSubjMesh, const FaceRange *pSubj,
	V3_F32 (* subjGetPos)(const void *, const Mesh *, const FaceRange *, I32)
);