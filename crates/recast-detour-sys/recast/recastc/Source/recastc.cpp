#include "recastc.h"
#include <DetourNavMeshQuery.h>
#include <DetourNavMeshBuilder.h>
#include <DetourAlloc.h>

#include <memory>
#include <assert.h>

struct recastc_Query
{
	dtNavMeshQuery *q;
	dtNavMesh* mesh;
	dtQueryFilter filter;
};

static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly);

/// Reference Doc:
/// * A discussion how to NavMesh from pre-cull triangles soup
/// 	* https://groups.google.com/forum/#!topic/recastnavigation/Tjq7G-KUxt8

static void write_error(const char* msg, recastc_Error* error) {
	assert(strlen(msg) < 255);
	strcpy(error->msg, msg);	
}

#define RETURN_ERROR(MSG)		do {  write_error(MSG, error); return 0;  } while(0)
#define POLYFLAGS_WALK			0x1

extern "C"
{	
	const char *recastc_version()
	{
		return "0.0.1";
	}

	/// Create Query Objects
	struct recastc_Query *recastc_create_query(recastc_NavMesh* qparam, recastc_Error* error)
	{
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));

		/// NOTE:
		/// NavMesh use unsigned short to store position data
		/// Which means we have to normalize a float to unsigned short
		int pm_vert_count = qparam->vert_count;							///< The number vertices in the polygon mesh. [Limit: >= 3]
		int pm_nvp = 3;												    ///< Number maximum number of vertices per polygon. [Limit: >= 3]
		int pm_polyCount = qparam->triangles_count;					    ///< Number of polygons in the mesh. [Limit: >= 1]		
		
		auto pm_polys = std::unique_ptr<unsigned short[]>(new unsigned short[pm_polyCount * 2 * pm_nvp]);  ///< The polygon data. [Size: #polyCount * 2 * #nvp]		
		memset(pm_polys.get(), 0, pm_polyCount * 2 * pm_nvp * sizeof(unsigned short));

		// fill the first part of the pm_polys with triangle indices
		for (int i = 0; i < pm_polyCount; i++)
		{
			pm_polys[i*pm_nvp*2+0] = qparam->indices[i*pm_nvp+0];
			pm_polys[i*pm_nvp*2+1] = qparam->indices[i*pm_nvp+1];
			pm_polys[i*pm_nvp*2+2] = qparam->indices[i*pm_nvp+2];
			pm_polys[i*pm_nvp*2+3] = 0;
			pm_polys[i*pm_nvp*2+4] = 0;
			pm_polys[i*pm_nvp*2+5] = 0;
		}

		if(!buildMeshAdjacency(pm_polys.get(), pm_polyCount, pm_vert_count, pm_nvp)) {
			RETURN_ERROR("dtCreateNavMeshdata failed!");
		}

		auto pm_polyFlags = std::unique_ptr<uint16_t[]>(new uint16_t[pm_polyCount]);		    ///< The user defined flags assigned to each polygon. [Size: #polyCount]
		memset(pm_polyFlags.get(), 0, sizeof(uint16_t) * pm_polyCount);		
		
		// Referece: Sample_SoloMesh.cpp Line:667
		for(auto i = 0; i < pm_polyCount; i++)
			pm_polyFlags[i] = POLYFLAGS_WALK;


		auto pm_polyAreas = std::unique_ptr<unsigned char[]>(new unsigned char[pm_polyCount]);		    ///< The user defined area ids assigned to each polygon. [Size: #polyCount]		
		// TODO(edwin):
		// We assume all polygon are in the same area
		memset(pm_polyAreas.get(), 0, sizeof(unsigned char) * pm_polyCount);		

		//< The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
		auto pm_verts_p = std::unique_ptr<uint16_t[]>(new uint16_t[pm_vert_count*3]);
		memcpy(pm_verts_p.get(), qparam->verts, sizeof(uint16_t) * pm_vert_count * 3);

		params.verts = pm_verts_p.get();
		params.vertCount = pm_vert_count;
		params.polys = pm_polys.get();
		params.polyCount = pm_polyCount;
		params.polyAreas = pm_polyAreas.get();
		params.polyFlags = pm_polyFlags.get();
		params.nvp = pm_nvp;
		params.detailMeshes = 0;
		params.offMeshConAreas = 0;
		params.offMeshConCount = 0;
		params.offMeshConDir = 0;
		params.offMeshConFlags = 0;
		params.offMeshConRad = 0;
		params.offMeshConUserID = 0;
		params.offMeshConVerts = 0;
		params.walkableHeight = qparam->walkable_height;
		params.walkableRadius = qparam->walkable_radius;
		params.walkableClimb = qparam->walkable_climb;
		params.cs = qparam->cell_size;
		params.ch = qparam->cell_height;
		memcpy(params.bmin, &qparam->bmin, sizeof(params.bmin));
		memcpy(params.bmax, &qparam->bmax, sizeof(params.bmax));
		params.buildBvTree = true;

		unsigned char* navData = 0;
		int navDataSize = 0;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
			RETURN_ERROR("dtCreateNavMeshdata failed!");
		}

		if(navData == NULL) {
			RETURN_ERROR("dtCreateNavMeshdata return null data");
		}
		if(navDataSize == 0) {
			RETURN_ERROR("dtCreateNavMeshdata return zero size data");
		}

		dtNavMesh *mesh = dtAllocNavMesh();

		dtStatus status = mesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			dtFreeNavMesh(mesh);

			RETURN_ERROR("Could not init Detour navmesh");
		}

		auto q = dtAllocNavMeshQuery();
		status = q->init(mesh, 1000);
		if (dtStatusFailed(status))
		{
			dtFreeNavMeshQuery(q);
			dtFreeNavMesh(mesh);		

			RETURN_ERROR("Could not init Detour navmesh");
		}
			
		auto query = new recastc_Query();
		query->q = q;
		query->mesh = mesh;

		return query;
	}

	void recastc_free_query(struct recastc_Query *query)
	{
		dtFreeNavMeshQuery(query->q);
		dtFreeNavMesh(query->mesh);		
		delete query;
	}

	int32_t recastc_find_nearest_poly(struct recastc_Query* query, 
        const recastc_NearestPolyInput* input, 
        recastc_NearestPolyResult* result,
        recastc_Error* error) 
	{
		assert(query);
		assert(query->q);
		assert(input);
		assert(result);		
		assert(sizeof(dtPolyRef) == sizeof(uint32_t));

		query->filter.setIncludeFlags(POLYFLAGS_WALK);
		dtStatus status = query->q->findNearestPoly(input->center, input->half_extents, &query->filter, &result->poly, result->pos);

		if (dtStatusFailed(status))
		{
			RETURN_ERROR("FAIL_TO_FIND_PATH");
		}

		if( dtStatusDetail(status, DT_INVALID_PARAM) )
		{
			RETURN_ERROR("INVALID_PARAM");
		}
		// check details
		if( dtStatusDetail(status, DT_BUFFER_TOO_SMALL) )
		{
			RETURN_ERROR("BUFFER_TOO_SMALL");
		}

		if( dtStatusDetail(status, DT_OUT_OF_NODES) )
		{
			RETURN_ERROR("OUT_OF_NODES");
		}

		if( dtStatusDetail(status, DT_PARTIAL_RESULT) )
		{
			RETURN_ERROR("PARTIAL_RESULT");
		}

		return 1;
	}

	int32_t recastc_find_closest_point(struct recastc_Query* query, 
        const recastc_ClosestPointInput* input,
        recastc_ClosestPointResult* result,
        recastc_Error* error)
	{
		assert(query);	
		assert(query->q);	
		assert(input);
		assert(result);

		query->filter.setIncludeFlags(POLYFLAGS_WALK);

		dtStatus status = query->q->closestPointOnPoly(input->poly, input->pos, result->pos, 0);

		if( dtStatusDetail(status, DT_INVALID_PARAM) )
		{
			RETURN_ERROR("Fail to find path: reason: [Invalid Param]");
		}		

		if (dtStatusFailed(status))
		{
			RETURN_ERROR("Fail to find closest point: reason[unknown]");
		}

	 	return 1;
	}

	

	// m_navQuery->closestPointOnPoly(m_startRef, m_spos, m_iterPos, 0);

	// int32_t recastc_find_path(struct recastc_Query* query, recastc_Error* error) {
	// 	assert(query);
	// 	assert(query->q);		

	// 	dtPolyRef startRef;
	// 	dtPolyRef endRef;

	// 	const float* startPos;
	// 	const float* endPos;

	// 	query->filter.setIncludeFlags(POLYFLAGS_WALK);

	// 	int pathCount;
	// 	int maxPath;

	// 	dtPolyRef path[maxPath];

		
		
	// 	/// Finds a path from the start polygon to the end polygon.
	// 	///  @param[in]		startRef	The refrence id of the start polygon.
	// 	///  @param[in]		endRef		The reference id of the end polygon.
	// 	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	// 	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	// 	///  @param[in]		filter		The polygon filter to apply to the query.
	// 	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	// 	///  							[(polyRef) * @p pathCount]
	// 	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	// 	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	

	// 	return 1;
	// }

	int32_t recastc_find_path(struct recastc_Query* query, 
        const recastc_PathInput* input,
        recastc_PathResult* result,
        recastc_Error* error)
	{
		assert(query);	
		assert(query->q);	
		assert(input);
		assert(result);

		query->filter.setIncludeFlags(POLYFLAGS_WALK);	

		int count = 0;

		assert(sizeof(result->path) / sizeof(uint32_t) == 100);
				
		dtStatus status = query->q->findPath(
			input->start_poly, 
			input->end_poly, 
			input->start_pos,
			input->end_pos,			
			&query->filter,
			result->path,
			&count,
			sizeof(result->path) / sizeof(uint32_t)
		);
		
		result->path_count = count;
		if (dtStatusFailed(status))
		{
			RETURN_ERROR("FAIL_TO_FIND_PATH");
		}

		if( dtStatusDetail(status, DT_INVALID_PARAM) )
		{
			RETURN_ERROR("INVALID_PARAM");
		}
		// check details
		if( dtStatusDetail(status, DT_BUFFER_TOO_SMALL) )
		{
			RETURN_ERROR("BUFFER_TOO_SMALL");
		}

		if( dtStatusDetail(status, DT_OUT_OF_NODES) )
		{
			RETURN_ERROR("OUT_OF_NODES");
		}

		if( dtStatusDetail(status, DT_PARTIAL_RESULT) )
		{
			RETURN_ERROR("PARTIAL_RESULT");
		}

		return 1;
	}
}

struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static const unsigned short RC_MESH_NULL_IDX = 0xffff;

/// Straight up copy & paste from RecastMesh.cpp
/// We assume `polys`
static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly)
{
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php
	
	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)dtAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), DT_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	rcEdge* edges = (rcEdge*)dtAlloc(sizeof(rcEdge)*maxEdgeCount, DT_ALLOC_TEMP);
	if (!edges)
	{
		dtFree(firstEdge);
		return false;
	}
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
					}
				}
			}
		}
	}
	
	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
	}
	
	dtFree(firstEdge);
	dtFree(edges);
	
	return true;
}
