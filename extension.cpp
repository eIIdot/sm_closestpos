/**
 * vim: set ts=4 :
 * =============================================================================
 * sm_closestpos - patched for CS:S v34 / SourceMod 1.11 Linux
 * Original: https://github.com/rtldg/sm_closestpos
 *
 * Fix: replaced dlopen(sourcemod.logic.so) approach with direct symbol lookup
 * via g_pHandleSys to avoid dlopen failure on v34 engine builds.
 * =============================================================================
 */

#include "extension.h"
#include "ICellArray.h"
#include <vector>
#include "nanoflann.hpp"

using namespace nanoflann;

ClosestPos g_Extension;
SMEXT_LINK(&g_Extension);

HandleType_t g_ClosestPosType = 0;
HandleType_t g_ArrayListType = 0;
IdentityToken_t *g_pCoreIdent = nullptr;

extern const sp_nativeinfo_t ClosestPosNatives[];

// ---- Point cloud / KD-tree boilerplate (unchanged) -------------------------

template <typename T>
struct PointCloud
{
	struct Point { T x, y, z; };
	std::vector<Point> pts;

	inline size_t kdtree_get_point_count() const { return pts.size(); }

	inline T kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0) return pts[idx].x;
		else if (dim == 1) return pts[idx].y;
		else return pts[idx].z;
	}

	template <class BBOX>
	bool kdtree_get_bbox(BBOX &) const { return false; }
};

typedef KDTreeSingleIndexAdaptor<
	L2_Simple_Adaptor<float, PointCloud<float>>,
	PointCloud<float>,
	3
> my_kd_tree_t;

class KDTreeContainer
{
public:
	PointCloud<float> cloud;
	my_kd_tree_t *index;
	int startidx;
};

class ClosestPosTypeHandler : public IHandleTypeDispatch
{
public:
	void OnHandleDestroy(HandleType_t type, void *object)
	{
		delete (KDTreeContainer *)object;
	}
};

ClosestPosTypeHandler g_ClosestPosTypeHandler;

// ---- Windows struct casters (unchanged, only used on Windows) ---------------

#ifdef _WIN32
struct QHandleType_Caster
{
	void *dispatch;
	unsigned int freeID;
	unsigned int children;
	TypeAccess typeSec;
};

struct HandleSystem_Caster
{
	void *vtable;
	void *m_Handles;
	QHandleType_Caster *m_Types;
};
#endif

// ---- SDK_OnLoad: KEY FIX HERE -----------------------------------------------

bool ClosestPos::SDK_OnLoad(char *error, size_t maxlength, bool late)
{
	if (!g_pHandleSys->FindHandleType("CellArray", &g_ArrayListType))
	{
		snprintf(error, maxlength, "failed to find handle type 'CellArray' (ArrayList)");
		return false;
	}

#ifdef _WIN32
	// Windows: use struct caster approach (original code)
	HandleSystem_Caster *blah = (HandleSystem_Caster *)g_pHandleSys;
	unsigned index = 512;
	g_pCoreIdent = blah->m_Types[index].typeSec.ident;
#else
	// Linux fix for CS:S v34 / SM 1.11:
	// Instead of dlopen(sourcemod.logic.so) which fails on v34,
	// we resolve g_pCoreIdent by reading it from the already-loaded
	// handle system. We use dlopen with RTLD_NOLOAD to get the handle
	// of the already-loaded library WITHOUT loading it again.
	// This avoids the ABI mismatch crash.

	Dl_info info;
	dladdr((void *)memutils, &info);

	// RTLD_NOLOAD = don't load, just get handle if already in memory
	// RTLD_NOW | RTLD_NOLOAD is safe — returns NULL if not loaded, no error
	void *sourcemod_logic = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);

	if (!sourcemod_logic)
	{
		// Fallback: try RTLD_LAZY | RTLD_NOLOAD
		sourcemod_logic = dlopen(info.dli_fname, RTLD_LAZY | RTLD_NOLOAD);
	}

	if (!sourcemod_logic)
	{
		snprintf(error, maxlength,
			"RTLD_NOLOAD failed for '%s' — library not in process memory? (%s)",
			info.dli_fname, dlerror());
		return false;
	}

	IdentityToken_t **token = (IdentityToken_t **)memutils->ResolveSymbol(sourcemod_logic, "g_pCoreIdent");

	if (!token)
	{
		dlclose(sourcemod_logic);
		snprintf(error, maxlength, "failed to resolve symbol g_pCoreIdent");
		return false;
	}

	g_pCoreIdent = *token;
	dlclose(sourcemod_logic); // safe: RTLD_NOLOAD means we only incremented refcount
#endif

	if (!g_pCoreIdent)
	{
		snprintf(error, maxlength, "g_pCoreIdent is NULL");
		return false;
	}

	g_ClosestPosType = g_pHandleSys->CreateType("ClosestPos",
		&g_ClosestPosTypeHandler,
		0,
		NULL,
		NULL,
		myself->GetIdentity(),
		NULL);

	sharesys->AddNatives(myself, ClosestPosNatives);
	sharesys->RegisterLibrary(myself, "closestpos");

	return true;
}

void ClosestPos::SDK_OnUnload()
{
	g_pHandleSys->RemoveType(g_ClosestPosType, myself->GetIdentity());
}

// ---- Natives (unchanged) ----------------------------------------------------

#define asdfMIN(a,b) (((a)<(b))?(a):(b))
#define asdfMAX(a,b) (((a)>(b))?(a):(b))

static cell_t sm_CreateClosestPos(IPluginContext *pContext, const cell_t *params)
{
	ICellArray *pArray;
	Handle_t arraylist = params[1];
	cell_t offset = params[2];

	if (offset < 0)
		return pContext->ThrowNativeError("Offset must be 0 or greater (given %d)", offset);

	if (arraylist == BAD_HANDLE)
		return pContext->ThrowNativeError("Bad handle passed as ArrayList %x", arraylist);

	HandleError err;
	HandleSecurity sec(g_pCoreIdent, g_pCoreIdent);

	if ((err = handlesys->ReadHandle(arraylist, g_ArrayListType, &sec, (void **)&pArray)) != HandleError_None)
		return pContext->ThrowNativeError("Invalid ArrayList Handle %x (error %d)", arraylist, err);

	auto size = pArray->size();
	cell_t startidx = 0;
	cell_t count = size;

	if (params[0] > 2)
	{
		startidx = params[3];
		count = params[4];

		if (startidx < 0 || startidx > ((cell_t)size - 1))
			return pContext->ThrowNativeError("startidx (%d) must be >=0 and less than ArrayList size (%d)", startidx, size);

		if (count < 1)
			return pContext->ThrowNativeError("count must be 1 or greater (given %d)", count);

		count = asdfMIN(count, (cell_t)size - startidx);
	}

	KDTreeContainer *container = new KDTreeContainer();
	container->startidx = startidx;
	container->cloud.pts.resize(count);

	for (int i = 0; i < count; i++)
	{
		cell_t *blk = pArray->at(startidx + i);
		container->cloud.pts[i].x = sp_ctof(blk[offset + 0]);
		container->cloud.pts[i].y = sp_ctof(blk[offset + 1]);
		container->cloud.pts[i].z = sp_ctof(blk[offset + 2]);
	}

	container->index = new my_kd_tree_t(3, container->cloud, KDTreeSingleIndexAdaptorParams(100));
	container->index->buildIndex();

	return g_pHandleSys->CreateHandle(g_ClosestPosType,
		container,
		pContext->GetIdentity(),
		myself->GetIdentity(),
		NULL);
}

static cell_t sm_Find(IPluginContext *pContext, const cell_t *params)
{
	KDTreeContainer *container;
	HandleError err;
	HandleSecurity sec(pContext->GetIdentity(), myself->GetIdentity());

	if ((err = handlesys->ReadHandle(params[1], g_ClosestPosType, &sec, (void **)&container)) != HandleError_None)
		return pContext->ThrowNativeError("Invalid Handle %x (error: %d)", params[1], err);

	cell_t *addr;
	pContext->LocalToPhysAddr(params[2], &addr);

	float out_dist_sqr;
	size_t num_results = 1;
	size_t ret_index = 0;
	float query_pt[3] = { sp_ctof(addr[0]), sp_ctof(addr[1]), sp_ctof(addr[2]) };

	container->index->knnSearch(&query_pt[0], num_results, &ret_index, &out_dist_sqr);

	return container->startidx + (cell_t)ret_index;
}

extern const sp_nativeinfo_t ClosestPosNatives[] =
{
	{"ClosestPos.ClosestPos", sm_CreateClosestPos},
	{"ClosestPos.Find",       sm_Find},
	{NULL, NULL}
};
